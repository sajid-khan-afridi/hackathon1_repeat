"""
Unit tests for PersonalizationService.
T072: Test profile context string generation for RAG chatbot.
"""

import pytest
from uuid import uuid4
from unittest.mock import AsyncMock
from app.services.personalization_service import PersonalizationService


class TestBuildProfileContext:
    """Test profile context string generation for RAG chatbot."""

    def test_build_profile_context_complete_profile(self):
        """Test context generation with complete user profile."""
        profile = {
            "experience_level": "intermediate",
            "ros_familiarity": "basic",
            "preferred_language": "python",
            "hardware_access": "physical_robot",
            "learning_goal": "career_transition",
        }

        context = PersonalizationService.build_profile_context(profile)

        # Verify all profile attributes are in context
        assert "intermediate" in context.lower()
        assert "basic" in context.lower()
        assert "python" in context.lower()
        assert "physical_robot" in context.lower() or "physical robot" in context.lower()
        assert "career_transition" in context.lower() or "career transition" in context.lower()

    def test_build_profile_context_beginner_profile(self):
        """Test context generation for beginner user."""
        profile = {
            "experience_level": "beginner",
            "ros_familiarity": "none",
            "preferred_language": "python",
            "hardware_access": "simulation_only",
            "learning_goal": "academic_research",
        }

        context = PersonalizationService.build_profile_context(profile)

        assert "beginner" in context.lower()
        assert "python" in context.lower()
        assert "simulation" in context.lower()

        # Verify context suggests appropriate complexity
        # Context should guide LLM to provide beginner-friendly answers
        assert any(
            word in context.lower()
            for word in ["simple", "basic", "beginner", "introductory", "fundamental"]
        )

    def test_build_profile_context_advanced_profile(self):
        """Test context generation for advanced user."""
        profile = {
            "experience_level": "advanced",
            "ros_familiarity": "proficient",
            "preferred_language": "cpp",
            "hardware_access": "physical_robot",
            "learning_goal": "academic_research",
        }

        context = PersonalizationService.build_profile_context(profile)

        assert "advanced" in context.lower()
        assert "proficient" in context.lower()
        assert "cpp" in context.lower() or "c++" in context.lower()

    def test_build_profile_context_default_profile(self):
        """Test context generation with default/minimal profile."""
        profile = {
            "experience_level": "intermediate",
            "ros_familiarity": "basic",
            "preferred_language": "python",
            "hardware_access": "simulation_only",
            "learning_goal": "personal_interest",
        }

        context = PersonalizationService.build_profile_context(profile)

        # Verify it generates valid context for default profile
        assert isinstance(context, str)
        assert len(context) > 0
        assert "intermediate" in context.lower()

    def test_build_profile_context_both_languages(self):
        """Test context generation with 'both' language preference."""
        profile = {
            "experience_level": "intermediate",
            "ros_familiarity": "basic",
            "preferred_language": "both",
            "hardware_access": "physical_robot",
            "learning_goal": "career_transition",
        }

        context = PersonalizationService.build_profile_context(profile)

        # Should mention both languages or indicate language flexibility
        assert "both" in context.lower() or "python" in context.lower() or "cpp" in context.lower()

    def test_build_profile_context_is_string(self):
        """Test that context is always a non-empty string."""
        profile = {
            "experience_level": "beginner",
            "ros_familiarity": "none",
            "preferred_language": "python",
            "hardware_access": "simulation_only",
            "learning_goal": "academic_research",
        }

        context = PersonalizationService.build_profile_context(profile)

        assert isinstance(context, str)
        assert len(context) > 50  # Should be reasonably descriptive

    def test_build_profile_context_guides_llm_behavior(self):
        """Test that context includes instructions for LLM."""
        profile = {
            "experience_level": "intermediate",
            "ros_familiarity": "basic",
            "preferred_language": "python",
            "hardware_access": "simulation_only",
            "learning_goal": "career_transition",
        }

        context = PersonalizationService.build_profile_context(profile)

        # Context should include guidance for LLM behavior
        # Check for instruction keywords
        instruction_keywords = [
            "adjust",
            "tailor",
            "provide",
            "use",
            "prefer",
            "focus",
            "explain",
            "assume",
        ]

        has_instructions = any(keyword in context.lower() for keyword in instruction_keywords)
        assert has_instructions, "Context should include LLM guidance instructions"


class TestHandleUnauthenticatedUsers:
    """Test default profile handling for unauthenticated users."""

    def test_default_profile_for_unauthenticated(self):
        """Test that unauthenticated users get default intermediate profile."""
        default_profile = {
            "experience_level": "intermediate",
            "ros_familiarity": "basic",
            "preferred_language": "python",
            "hardware_access": "simulation_only",
            "learning_goal": "personal_interest",
        }

        context = PersonalizationService.build_profile_context(default_profile)

        # Verify default intermediate assumptions
        assert "intermediate" in context.lower()
        assert "python" in context.lower()


class TestProfileContextVariations:
    """Test profile context variations for different user types."""

    def test_context_differs_by_experience_level(self):
        """Test that context differs meaningfully by experience level."""
        beginner_profile = {
            "experience_level": "beginner",
            "ros_familiarity": "none",
            "preferred_language": "python",
            "hardware_access": "simulation_only",
            "learning_goal": "academic_research",
        }

        advanced_profile = {
            "experience_level": "advanced",
            "ros_familiarity": "proficient",
            "preferred_language": "cpp",
            "hardware_access": "physical_robot",
            "learning_goal": "academic_research",
        }

        beginner_context = PersonalizationService.build_profile_context(beginner_profile)
        advanced_context = PersonalizationService.build_profile_context(advanced_profile)

        # Contexts should be different
        assert beginner_context != advanced_context

        # Beginner context should emphasize simplicity
        assert "beginner" in beginner_context.lower()

        # Advanced context should allow complexity
        assert "advanced" in advanced_context.lower()

    def test_context_preserves_hardware_preference(self):
        """Test that hardware preference is preserved in context."""
        physical_profile = {
            "experience_level": "intermediate",
            "ros_familiarity": "basic",
            "preferred_language": "python",
            "hardware_access": "physical_robot",
            "learning_goal": "career_transition",
        }

        simulation_profile = {
            "experience_level": "intermediate",
            "ros_familiarity": "basic",
            "preferred_language": "python",
            "hardware_access": "simulation_only",
            "learning_goal": "career_transition",
        }

        physical_context = PersonalizationService.build_profile_context(physical_profile)
        simulation_context = PersonalizationService.build_profile_context(simulation_profile)

        # Physical context should mention hardware
        assert "physical" in physical_context.lower() or "robot" in physical_context.lower()

        # Simulation context should mention simulation
        assert "simulation" in simulation_context.lower()

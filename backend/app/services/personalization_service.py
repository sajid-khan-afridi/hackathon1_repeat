"""
Personalization Service for Phase 4B Personalization Engine.
Orchestrates personalization logic including profile context generation for RAG chatbot.
"""

import logging
from uuid import UUID
from typing import Optional, Dict, Any
import asyncpg

logger = logging.getLogger(__name__)


class PersonalizationService:
    """Service for personalization orchestration operations."""

    @staticmethod
    def build_profile_context(
        user_profile: Optional[Dict[str, Any]],
        skill_level: Optional[str] = None,
    ) -> str:
        """
        Build profile context string for RAG chatbot system message.

        This method generates a structured system message that instructs the LLM
        to adapt its response complexity, code language, and hardware references
        based on the user's profile attributes.

        Args:
            user_profile: User profile dictionary with keys:
                - experience_level: 'beginner', 'intermediate', 'advanced'
                - ros_familiarity: 'none', 'basic', 'proficient'
                - preferred_language: 'python', 'cpp', 'both'
                - hardware_access: 'simulation_only', 'basic_hardware', 'advanced_hardware'
                - learning_goal: 'hobby', 'academic_research', 'career_transition', 'skill_upgrade'
            skill_level: Computed skill level ('beginner', 'intermediate', 'advanced')

        Returns:
            Formatted system message string with profile context

        Examples:
            >>> profile = {
            ...     "experience_level": "intermediate",
            ...     "ros_familiarity": "basic",
            ...     "preferred_language": "python",
            ...     "hardware_access": "simulation_only",
            ...     "learning_goal": "academic_research"
            ... }
            >>> context = PersonalizationService.build_profile_context(profile, "intermediate")
            >>> "intermediate skill level" in context
            True
        """
        # Default profile for unauthenticated users
        if not user_profile:
            logger.debug("No user profile provided, using default intermediate profile")
            return """You are a helpful robotics tutor for the robotics textbook chatbot.

User Profile: Not authenticated (default intermediate level)

Response Guidelines:
- Use intermediate complexity suitable for users with some programming experience
- Provide code examples in both Python and C++ when relevant
- Assume access to simulation tools like Gazebo and Isaac Sim
- Explain concepts clearly with practical examples

Remember to:
1. Cite sources using [Source N] references from the retrieved context
2. Provide accurate, textbook-based answers only
3. Maintain a friendly, educational tone"""

        # Extract profile attributes with defaults
        experience_level = user_profile.get("experience_level", "intermediate")
        ros_familiarity = user_profile.get("ros_familiarity", "basic")
        preferred_language = user_profile.get("preferred_language", "python")
        hardware_access = user_profile.get("hardware_access", "simulation_only")
        learning_goal = user_profile.get("learning_goal", "academic_research")

        # Use provided skill_level or default to intermediate
        effective_skill_level = skill_level or "intermediate"

        # Build complexity instruction based on skill level
        complexity_map = {
            "beginner": "Use beginner-friendly language with step-by-step explanations. Avoid complex technical jargon unless explained. Provide more context and foundational concepts.",
            "intermediate": "Use intermediate complexity suitable for users with some experience. Balance theory and practice. Assume familiarity with basic programming concepts.",
            "advanced": "Use advanced technical depth suitable for experienced practitioners. You can reference complex concepts, research papers, and optimization techniques.",
        }
        complexity_instruction = complexity_map.get(
            effective_skill_level,
            complexity_map["intermediate"],
        )

        # Build code language instruction
        language_map = {
            "python": "Provide code examples primarily in Python. Only mention C++ alternatives if directly relevant.",
            "cpp": "Provide code examples primarily in C++. Only mention Python alternatives if directly relevant.",
            "both": "Provide code examples in both Python and C++ when relevant, showing equivalent implementations.",
        }
        language_instruction = language_map.get(
            preferred_language, language_map["python"]
        )

        # Build hardware instruction
        hardware_map = {
            "simulation_only": "Focus on simulation-based examples using Gazebo and Isaac Sim. Explain how concepts apply to simulation environments.",
            "basic_hardware": "Include practical hardware examples for basic robotics platforms (Arduino, basic ROS robots). Balance simulation and physical implementation.",
            "advanced_hardware": "Reference advanced hardware platforms and real-world robotics applications. Discuss production-level considerations and hardware constraints.",
        }
        hardware_instruction = hardware_map.get(
            hardware_access, hardware_map["simulation_only"]
        )

        # Build learning goal instruction
        goal_map = {
            "hobby": "Emphasize practical, hands-on projects and fun applications. Keep explanations accessible and project-focused.",
            "academic_research": "Emphasize theoretical foundations, research papers, and academic rigor. Reference relevant publications when appropriate.",
            "career_transition": "Focus on industry-relevant skills, best practices, and practical job market competencies. Highlight real-world applications.",
            "skill_upgrade": "Build on existing knowledge efficiently. Focus on advanced techniques and optimization strategies.",
        }
        goal_instruction = goal_map.get(learning_goal, goal_map["academic_research"])

        # Assemble complete profile context
        profile_context = f"""You are a helpful robotics tutor for the robotics textbook chatbot.

User Profile:
- Skill Level: {effective_skill_level}
- Experience Level: {experience_level}
- ROS Familiarity: {ros_familiarity}
- Preferred Language: {preferred_language}
- Hardware Access: {hardware_access}
- Learning Goal: {learning_goal}

Response Guidelines:
{complexity_instruction}

{language_instruction}

{hardware_instruction}

{goal_instruction}

Remember to:
1. Cite sources using [Source N] references from the retrieved context
2. Provide accurate, textbook-based answers only
3. Maintain a friendly, educational tone
4. Adapt your response complexity and examples to match the user's profile"""

        logger.debug(
            f"Built profile context for skill_level={effective_skill_level}, "
            f"language={preferred_language}, hardware={hardware_access}"
        )

        return profile_context

    @staticmethod
    async def get_user_profile_with_classification(
        user_id: UUID, conn: asyncpg.Connection
    ) -> Dict[str, Any]:
        """
        Retrieve user profile along with skill level classification.

        Args:
            user_id: UUID of the user
            conn: Database connection

        Returns:
            Dictionary with profile attributes and skill_level, or empty dict if not found

        Raises:
            asyncpg.PostgresError: If database operation fails
        """
        try:
            # Fetch user profile
            profile_row = await conn.fetchrow(
                """
                SELECT experience_level, ros_familiarity, hardware_access,
                       learning_goal, preferred_language
                FROM user_profiles
                WHERE user_id = $1
                """,
                user_id,
            )

            if not profile_row:
                logger.warning(f"Profile not found for user {user_id}")
                return {}

            # Fetch skill level classification
            classification_row = await conn.fetchrow(
                """
                SELECT skill_level
                FROM skill_level_classifications
                WHERE user_id = $1
                """,
                user_id,
            )

            # Build profile dictionary
            profile_data = {
                "experience_level": profile_row["experience_level"],
                "ros_familiarity": profile_row["ros_familiarity"],
                "hardware_access": profile_row["hardware_access"],
                "learning_goal": profile_row["learning_goal"],
                "preferred_language": profile_row["preferred_language"],
                "skill_level": (
                    classification_row["skill_level"]
                    if classification_row
                    else "intermediate"
                ),
            }

            logger.debug(f"Retrieved profile for user {user_id}: skill_level={profile_data['skill_level']}")
            return profile_data

        except Exception as e:
            logger.error(
                f"Failed to retrieve profile for user {user_id}: {str(e)}",
                exc_info=True,
            )
            # Return empty dict on error to fall back to default profile
            return {}

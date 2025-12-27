import React from 'react';
import PersonalizedSection from '@site/src/components/PersonalizedSection';
import TechnicalTerm from '@site/src/components/TechnicalTerm';
import { GlossaryTooltip } from '@site/src/components/GlossaryTooltip';

// Export custom components for use in MDX files
export default {
  PersonalizedSection,
  TechnicalTerm,
  // Phase 5: Translation Feature - T043
  // GlossaryTooltip wraps technical terms with bilingual hover tooltips
  GlossaryTooltip,
  // Alias for convenience in MDX
  Term: GlossaryTooltip,
};

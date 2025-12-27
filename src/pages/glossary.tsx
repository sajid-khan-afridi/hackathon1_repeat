/**
 * Glossary Page - Bilingual Technical Terminology
 * Phase 5: Translation Feature - T042
 *
 * Dedicated page for the searchable bilingual glossary with
 * English and Urdu technical terms for robotics and AI.
 */

import React from 'react';
import Layout from '@theme/Layout';
import { LanguageProvider } from '@site/src/context/LanguageContext';
import { GlossaryPage as GlossaryContent } from '@site/src/components/GlossaryPage';
import styles from './glossary.module.css';

export default function GlossaryPage(): JSX.Element {
  return (
    <Layout
      title="Technical Glossary"
      description="Bilingual glossary of robotics and AI terminology in English and Urdu"
    >
      <LanguageProvider>
        <div className={styles.glossaryPage}>
          <GlossaryContent />
        </div>
      </LanguageProvider>
    </Layout>
  );
}

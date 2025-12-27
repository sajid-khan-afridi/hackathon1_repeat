/**
 * GlossaryPage Component Tests
 * Phase 5: Translation Feature - T035
 */

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { GlossaryPage } from './index';
import { LanguageContext } from '../../context/LanguageContext';
import type { LanguageCode } from '../../types/translation';
import type { GlossaryTerm, GlossaryCategory } from '../../types/glossary';

// Mock glossary data
const mockTerms: GlossaryTerm[] = [
  {
    id: 'ros-2',
    english: 'ROS 2',
    urduTransliteration: 'آر او ایس ٹو',
    definition: 'Robot Operating System version 2, a middleware for robotics.',
    definitionUrdu: 'روبوٹ آپریٹنگ سسٹم ورژن 2، روبوٹکس کے لیے مڈل ویئر۔',
    category: 'Robotics Frameworks',
    relatedTerms: ['urdf'],
  },
  {
    id: 'urdf',
    english: 'URDF',
    urduTransliteration: 'یو آر ڈی ایف',
    definition: 'Unified Robot Description Format for robot models.',
    definitionUrdu: 'روبوٹ ماڈلز کے لیے یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ۔',
    category: 'Robotics Frameworks',
    relatedTerms: [],
  },
  {
    id: 'gazebo',
    english: 'Gazebo',
    urduTransliteration: 'گزیبو',
    definition: 'A 3D robot simulator.',
    definitionUrdu: 'ایک 3D روبوٹ سمیولیٹر۔',
    category: 'Simulation',
    relatedTerms: [],
  },
  {
    id: 'python',
    english: 'Python',
    urduTransliteration: 'پائتھون',
    definition: 'A programming language widely used in robotics.',
    definitionUrdu: 'ایک پروگرامنگ زبان جو روبوٹکس میں وسیع پیمانے پر استعمال ہوتی ہے۔',
    category: 'Programming Languages',
    relatedTerms: [],
  },
];

// Mock the useGlossary hook with stateful implementation
const mockSetSearchQuery = jest.fn();
const mockSetCategoryFilter = jest.fn();

jest.mock('../../hooks/useGlossary', () => ({
  useGlossary: () => {
    const [searchQuery, setSearchQueryState] = React.useState('');
    const [categoryFilter, setCategoryFilterState] = React.useState<string | null>(null);

    const setSearchQuery = (query: string) => {
      setSearchQueryState(query);
      mockSetSearchQuery(query);
    };

    const setCategoryFilter = (category: string | null) => {
      setCategoryFilterState(category);
      mockSetCategoryFilter(category);
    };

    let filteredTerms = mockTerms;

    if (categoryFilter) {
      filteredTerms = filteredTerms.filter(t => t.category === categoryFilter);
    }

    if (searchQuery.trim()) {
      const query = searchQuery.toLowerCase();
      filteredTerms = filteredTerms.filter(t =>
        t.english.toLowerCase().includes(query) ||
        t.urduTransliteration.includes(query) ||
        t.definition.toLowerCase().includes(query)
      );
    }

    return {
      terms: mockTerms,
      filteredTerms,
      searchQuery,
      setSearchQuery,
      categoryFilter,
      setCategoryFilter,
      categories: ['Middleware', 'Simulation', 'Standards', 'Programming'],
      getTermById: (id: string) => mockTerms.find(t => t.id === id),
      getRelatedTerms: (id: string) => {
        const term = mockTerms.find(t => t.id === id);
        if (!term?.relatedTerms) return [];
        return term.relatedTerms
          .map(rid => mockTerms.find(t => t.id === rid))
          .filter(Boolean);
      },
      hasTerm: (id: string) => mockTerms.some(t => t.id === id),
      isLoading: false,
      error: null,
    };
  },
}));

// Custom render with LanguageContext
const renderWithLanguage = (
  ui: React.ReactElement,
  language: LanguageCode = 'en'
) => {
  return render(
    <LanguageContext.Provider
      value={{
        language,
        direction: language === 'ur' ? 'rtl' : 'ltr',
        config: {
          code: language,
          label: language === 'ur' ? 'اردو' : 'English',
          direction: language === 'ur' ? 'rtl' : 'ltr',
          htmlLang: language === 'ur' ? 'ur-PK' : 'en-US',
        },
        setLanguage: jest.fn(),
        hasTranslationError: false,
        setTranslationError: jest.fn(),
        isLoading: false,
      }}
    >
      {ui}
    </LanguageContext.Provider>
  );
};

describe('GlossaryPage', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  describe('Rendering', () => {
    it('renders page title', () => {
      renderWithLanguage(<GlossaryPage />);
      expect(screen.getByRole('heading', { level: 1 })).toHaveTextContent('Technical Glossary');
    });

    it('renders subtitle', () => {
      renderWithLanguage(<GlossaryPage />);
      expect(screen.getByText(/Bilingual robotics and AI terminology/i)).toBeInTheDocument();
    });

    it('renders search input', () => {
      renderWithLanguage(<GlossaryPage />);
      expect(screen.getByPlaceholderText(/Search terms/i)).toBeInTheDocument();
    });

    it('renders category filter dropdown', () => {
      renderWithLanguage(<GlossaryPage />);
      expect(screen.getByRole('combobox')).toBeInTheDocument();
    });

    it('renders term cards', () => {
      renderWithLanguage(<GlossaryPage />);
      expect(screen.getByText('ROS 2')).toBeInTheDocument();
      expect(screen.getByText('URDF')).toBeInTheDocument();
      expect(screen.getByText('Gazebo')).toBeInTheDocument();
      expect(screen.getByText('Python')).toBeInTheDocument();
    });

    it('renders term count', () => {
      renderWithLanguage(<GlossaryPage />);
      expect(screen.getByText(/4 terms/i)).toBeInTheDocument();
    });
  });

  describe('Urdu Language Mode', () => {
    it('renders Urdu title when language is Urdu', () => {
      renderWithLanguage(<GlossaryPage />, 'ur');
      expect(screen.getByRole('heading', { level: 1 })).toHaveTextContent('تکنیکی اصطلاحات');
    });

    it('renders Urdu placeholder when language is Urdu', () => {
      renderWithLanguage(<GlossaryPage />, 'ur');
      expect(screen.getByPlaceholderText(/اصطلاحات تلاش کریں/i)).toBeInTheDocument();
    });

    it('shows Urdu term count in Urdu mode', () => {
      renderWithLanguage(<GlossaryPage />, 'ur');
      expect(screen.getByText(/اصطلاحات/i)).toBeInTheDocument();
    });
  });

  describe('Search Functionality', () => {
    it('updates search on input change', async () => {
      const user = userEvent.setup();
      renderWithLanguage(<GlossaryPage />);

      const searchInput = screen.getByPlaceholderText(/Search terms/i);
      await user.type(searchInput, 'ROS');

      expect(searchInput).toHaveValue('ROS');
    });

    it('filters terms when searching', async () => {
      const user = userEvent.setup();
      renderWithLanguage(<GlossaryPage />);

      const searchInput = screen.getByPlaceholderText(/Search terms/i);
      await user.type(searchInput, 'Robot Operating');

      await waitFor(() => {
        expect(screen.getByText('ROS 2')).toBeInTheDocument();
      });
    });

    it('shows no results message when no terms match', async () => {
      const user = userEvent.setup();
      renderWithLanguage(<GlossaryPage />);

      const searchInput = screen.getByPlaceholderText(/Search terms/i);
      await user.type(searchInput, 'xxxxxxnonexistent');

      await waitFor(() => {
        expect(screen.getByText(/No terms found/i)).toBeInTheDocument();
      });
    });

    it('has accessible label for search', () => {
      renderWithLanguage(<GlossaryPage />);
      const searchInput = screen.getByPlaceholderText(/Search terms/i);
      expect(searchInput).toHaveAccessibleName();
    });
  });

  describe('Category Filter', () => {
    it('shows all categories in dropdown', async () => {
      const user = userEvent.setup();
      renderWithLanguage(<GlossaryPage />);

      const dropdown = screen.getByRole('combobox');
      expect(dropdown).toBeInTheDocument();

      // Options should exist
      expect(screen.getByRole('option', { name: /All categories/i })).toBeInTheDocument();
      expect(screen.getByRole('option', { name: /Middleware/i })).toBeInTheDocument();
      expect(screen.getByRole('option', { name: /Simulation/i })).toBeInTheDocument();
      expect(screen.getByRole('option', { name: /Standards/i })).toBeInTheDocument();
    });

    it('filters by category when selected', async () => {
      const user = userEvent.setup();
      renderWithLanguage(<GlossaryPage />);

      const dropdown = screen.getByRole('combobox');
      await user.selectOptions(dropdown, 'Middleware');

      await waitFor(() => {
        expect(mockSetCategoryFilter).toHaveBeenCalledWith('Middleware');
      });
    });

    it('has accessible label for category filter', () => {
      renderWithLanguage(<GlossaryPage />);
      const dropdown = screen.getByRole('combobox');
      expect(dropdown).toHaveAccessibleName();
    });
  });

  describe('Clear Filters', () => {
    it('shows clear button when filters are active', async () => {
      const user = userEvent.setup();
      renderWithLanguage(<GlossaryPage />);

      const searchInput = screen.getByPlaceholderText(/Search terms/i);
      await user.type(searchInput, 'test');

      expect(screen.getByRole('button', { name: /Clear filters/i })).toBeInTheDocument();
    });

    it('hides clear button when no filters active', () => {
      renderWithLanguage(<GlossaryPage />);
      expect(screen.queryByRole('button', { name: /Clear filters/i })).not.toBeInTheDocument();
    });

    it('clears all filters when clicked', async () => {
      const user = userEvent.setup();
      renderWithLanguage(<GlossaryPage />);

      const searchInput = screen.getByPlaceholderText(/Search terms/i);
      await user.type(searchInput, 'test');

      const clearButton = screen.getByRole('button', { name: /Clear filters/i });
      await user.click(clearButton);

      await waitFor(() => {
        expect(mockSetSearchQuery).toHaveBeenCalledWith('');
        expect(mockSetCategoryFilter).toHaveBeenCalledWith(null);
      });
    });
  });

  describe('Term Cards', () => {
    it('displays English term name', () => {
      renderWithLanguage(<GlossaryPage />);
      expect(screen.getByRole('heading', { name: /ROS 2/i })).toBeInTheDocument();
    });

    it('displays Urdu transliteration', () => {
      renderWithLanguage(<GlossaryPage />);
      expect(screen.getByText('آر او ایس ٹو')).toBeInTheDocument();
    });

    it('displays English definition', () => {
      renderWithLanguage(<GlossaryPage />);
      expect(screen.getByText(/Robot Operating System version 2/i)).toBeInTheDocument();
    });

    it('displays Urdu definition', () => {
      renderWithLanguage(<GlossaryPage />);
      expect(screen.getByText(/روبوٹ آپریٹنگ سسٹم ورژن 2/)).toBeInTheDocument();
    });

    it('displays category badge', () => {
      renderWithLanguage(<GlossaryPage />);
      expect(screen.getByText('Middleware')).toBeInTheDocument();
    });

    it('displays related terms when available', () => {
      renderWithLanguage(<GlossaryPage />);
      // ROS 2 has URDF as related term
      expect(screen.getByText(/Related terms/i)).toBeInTheDocument();
      expect(screen.getAllByText('urdf').length).toBeGreaterThanOrEqual(1);
    });

    it('uses list role for terms list', () => {
      renderWithLanguage(<GlossaryPage />);
      expect(screen.getByRole('list')).toBeInTheDocument();
    });

    it('uses listitem for each term', () => {
      renderWithLanguage(<GlossaryPage />);
      const items = screen.getAllByRole('listitem');
      expect(items.length).toBeGreaterThanOrEqual(4);
    });
  });

  describe('Loading State', () => {
    it('shows loading spinner when loading', () => {
      // Override useGlossary for this test
      const originalUseGlossary = jest.requireMock('../../hooks/useGlossary').useGlossary;
      jest.requireMock('../../hooks/useGlossary').useGlossary = () => ({
        ...originalUseGlossary(),
        isLoading: true,
      });

      renderWithLanguage(<GlossaryPage />);
      expect(screen.getByText(/Loading/i)).toBeInTheDocument();

      // Restore
      jest.requireMock('../../hooks/useGlossary').useGlossary = originalUseGlossary;
    });
  });

  describe('Error State', () => {
    it('shows error message when error occurs', () => {
      // Override useGlossary for this test
      const originalUseGlossary = jest.requireMock('../../hooks/useGlossary').useGlossary;
      jest.requireMock('../../hooks/useGlossary').useGlossary = () => ({
        ...originalUseGlossary(),
        error: new Error('Failed to load'),
      });

      renderWithLanguage(<GlossaryPage />);
      expect(screen.getByText(/Error loading glossary/i)).toBeInTheDocument();

      // Restore
      jest.requireMock('../../hooks/useGlossary').useGlossary = originalUseGlossary;
    });
  });

  describe('Accessibility', () => {
    it('has accessible page heading', () => {
      renderWithLanguage(<GlossaryPage />);
      expect(screen.getByRole('heading', { level: 1 })).toBeInTheDocument();
    });

    it('has accessible search input', () => {
      renderWithLanguage(<GlossaryPage />);
      const searchInput = screen.getByPlaceholderText(/Search terms/i);
      expect(searchInput).toHaveAttribute('aria-label');
    });

    it('has accessible category dropdown', () => {
      renderWithLanguage(<GlossaryPage />);
      const dropdown = screen.getByRole('combobox');
      expect(dropdown).toHaveAttribute('aria-label');
    });

    it('uses semantic list for terms', () => {
      renderWithLanguage(<GlossaryPage />);
      expect(screen.getByRole('list')).toBeInTheDocument();
    });

    it('has heading for each term', () => {
      renderWithLanguage(<GlossaryPage />);
      const headings = screen.getAllByRole('heading', { level: 2 });
      expect(headings.length).toBeGreaterThanOrEqual(4);
    });
  });

  describe('RTL Support', () => {
    it('works in RTL mode', () => {
      renderWithLanguage(<GlossaryPage />, 'ur');
      expect(screen.getByRole('heading', { level: 1 })).toBeInTheDocument();
    });

    it('renders all terms in RTL mode', () => {
      renderWithLanguage(<GlossaryPage />, 'ur');
      expect(screen.getByText('ROS 2')).toBeInTheDocument();
      expect(screen.getByText('URDF')).toBeInTheDocument();
    });
  });

  describe('Initial Props', () => {
    it('accepts initial query prop', async () => {
      renderWithLanguage(<GlossaryPage initialQuery="Gazebo" />);

      await waitFor(() => {
        expect(mockSetSearchQuery).toHaveBeenCalledWith('Gazebo');
      });
    });

    it('accepts initial category prop', async () => {
      renderWithLanguage(<GlossaryPage initialCategory={'Simulation' as GlossaryCategory} />);

      await waitFor(() => {
        expect(mockSetCategoryFilter).toHaveBeenCalledWith('Simulation');
      });
    });
  });

  describe('Responsive Design', () => {
    it('renders in container', () => {
      const { container } = renderWithLanguage(<GlossaryPage />);
      expect(container.querySelector('.container')).toBeInTheDocument();
    });
  });
});

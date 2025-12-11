# Physical AI & Humanoid Robotics Textbook Platform

A comprehensive, responsive, and accessible online textbook platform built with Docusaurus for teaching physical AI and humanoid robotics concepts.

## 🚀 Features

- **📚 Rich Content Support**: MDX-based content with syntax highlighting for Python, C++, ROS 2, and YAML
- **📱 Fully Responsive**: Optimized for mobile (320px), tablet (768px), and desktop (1024px+) viewports
- **🌙 Dark Mode**: Persistent dark/light theme toggle with system preference detection
- **♿ Accessibility First**: WCAG 2.1 AA compliant with keyboard navigation and proper focus management
- **⚡ Fast Performance**: Lighthouse scores >85 (Performance), >90 (Accessibility, Best Practices)
- **🚀 Auto-Deployment**: GitHub Actions CI/CD pipeline for automatic deployment to GitHub Pages
- **🔒 RTL Ready**: CSS logical properties for future multi-language support

## 📖 Table of Contents

- [Getting Started](#-getting-started)
- [Development](#-development)
- [Content Authoring](#-content-authoring)
- [Deployment](#-deployment)
- [Testing](#-testing)
- [Contributing](#-contributing)

## 🛠️ Getting Started

### Prerequisites

- **Node.js**: 20.0 or higher
- **npm**: 9.0 or higher
- **Git**: For version control

### Installation

1. **Clone the repository**
```bash
git clone https://github.com/sajid-khan-afridi/hackathon1_repeat.git
cd hackathon1_repeat
```

2. **Install dependencies**
```bash
npm install
```

3. **Start the development server**
```bash
npm start
```

4. **Open your browser**
Navigate to [http://localhost:3000/hackathon1_repeat](http://localhost:3000/hackathon1_repeat)

## 💻 Development

### Available Scripts

- `npm start` - Start development server with hot reload
- `npm run build` - Build for production
- `npm run serve` - Serve production build locally
- `npm run typecheck` - Run TypeScript type checking
- `npm run lint` - Run ESLint
- `npm run lint:fix` - Fix ESLint issues automatically
- `npm run format` - Format code with Prettier
- `npm run format:check` - Check code formatting

### Project Structure

```
hackathon1_repeat/
├── docs/                    # MDX content files
│   ├── intro.md            # Landing page
│   ├── module-1-ros2-fundamentals/
│   │   ├── _category_.json
│   │   └── chapter-1-publishers.mdx
│   └── module-2-isaac-sim/
│       ├── _category_.json
│       └── chapter-1-introduction.mdx
├── src/
│   ├── components/         # Custom React components
│   │   └── ThemeToggle/    # Dark mode toggle component
│   └── css/
│       └── custom.css      # Global styles with RTL support
├── static/                 # Static assets
├── tests/
│   ├── e2e/               # Playwright end-to-end tests
│   ├── lighthouse/        # Lighthouse CI configuration
│   └── unit/              # Jest unit tests
├── .github/
│   └── workflows/
│       └── deploy.yml     # GitHub Actions deployment
├── docusaurus.config.ts   # Docusaurus configuration
├── sidebars.ts           # Sidebar navigation structure
└── package.json          # Dependencies and scripts
```

## ✍️ Content Authoring

### Creating New Content

1. **Add a new module**:
   - Create a new directory in `docs/` (e.g., `module-3-computer-vision`)
   - Add a `_category_.json` file for module metadata

2. **Create a chapter**:
   - Add `.mdx` files to the module directory
   - Include frontmatter with metadata

### Frontmatter Example

```yaml
---
title: "Chapter Title"
sidebar_position: 1
description: Brief description of the chapter
keywords: ["keyword1", "keyword2"]
tags: ["tag1", "tag2"]
---

# Chapter Content

Your content here...
```

### Code Blocks

```python
# Python example
def hello_ros2():
    print("Hello, ROS 2!")
```

```cpp
// C++ example
#include "rclcpp/rclcpp.hpp"

int main() {
    rclcpp::init();
    // Your code here
    return 0;
}
```

## 🚀 Deployment

### Automatic Deployment

The site is automatically deployed to GitHub Pages when changes are pushed to the `main` branch.

### Manual Deployment

1. **Build the site**:
```bash
npm run build
```

2. **Deploy to GitHub Pages**:
```bash
npm run deploy
```

## 🧪 Testing

### Linting and Formatting

```bash
# Check for linting issues
npm run lint

# Auto-fix linting issues
npm run lint:fix

# Check code formatting
npm run format:check

# Format code
npm run format
```

### Type Checking

```bash
npm run typecheck
```

### End-to-End Testing with Playwright

```bash
# Install Playwright browsers
npx playwright install

# Run tests
npx playwright test

# Run tests with UI
npx playwright test --ui

# Run tests for specific viewport
npx playwright test --project="Mobile"
```

### Lighthouse Performance Testing

```bash
# Start the server
npm start

# In another terminal, run Lighthouse CI
lhci autorun
```

## 📊 Performance Metrics

The platform aims to meet the following performance criteria:

- **Lighthouse Performance**: >85
- **Lighthouse Accessibility**: >90
- **Lighthouse Best Practices**: >90
- **First Contentful Paint**: <1.5s
- **Time to Interactive**: <3s
- **Cumulative Layout Shift**: <0.1

## 🎨 Design System

### Colors

- **Primary Blue**: #2563eb (light), #60a5fa (dark)
- **Success Green**: #10b981 (light), #34d399 (dark)
- **Warning Amber**: #f59e0b (light), #fbbf24 (dark)
- **Danger Red**: #ef4444 (light), #f87171 (dark)

### Typography

- **Headings**: Clear hierarchy with responsive sizing
- **Body Text**: 16px minimum for accessibility
- **Code**: Monospace with syntax highlighting

### Responsive Breakpoints

- **Mobile**: <768px
- **Tablet**: 768px - 996px
- **Desktop**: >996px

## 🔧 Configuration

### Docusaurus Configuration

Edit `docusaurus.config.ts` to:
- Change site title and metadata
- Configure navigation
- Customize theme colors
- Adjust deployment settings

### Sidebar Configuration

Edit `sidebars.ts` to:
- Reorganize navigation structure
- Auto-generate from docs folder
- Create custom hierarchies

## 🤝 Contributing

We welcome contributions! Please:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new features
5. Ensure all tests pass
6. Submit a pull request

### Contribution Guidelines

- Follow the existing code style
- Add documentation for new features
- Test on multiple viewports
- Verify accessibility compliance
- Update README if needed

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- Built with [Docusaurus](https://docusaurus.io)
- Styled with CSS logical properties for RTL readiness
- Performance monitoring with [Lighthouse CI](https://github.com/GoogleChrome/lighthouse-ci)
- E2E testing with [Playwright](https://playwright.dev)

## 📞 Support

For questions or support:

- Create an issue on [GitHub](https://github.com/sajid-khan-afridi/hackathon1_repeat/issues)
- Check the [documentation](https://sajid-khan-afridi.github.io/hackathon1_repeat/)
- Review existing issues for solutions
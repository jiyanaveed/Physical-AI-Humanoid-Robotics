# Data Model: Advanced UI/UX Redesign for Spec-Driven Docusaurus Book

## Key Entities

### Visual Design System
**Description**: The collection of design tokens, components, and styling rules that create the cohesive visual identity of the book
**Fields**:
- `color_palette`: Object containing primary, secondary, and accent colors for light and dark themes
- `typography_scale`: Array of font sizes following a modular scale system
- `spacing_system`: Array of spacing units following a consistent ratio
- `breakpoints`: Object defining responsive breakpoints for different screen sizes
- `component_styles`: Object containing style definitions for all reusable UI components
- `design_tokens`: Object containing reusable design properties (border radius, shadows, etc.)

**Validation Rules**:
- Color contrast ratios must meet WCAG 2.1 AA standards (4.5:1 for normal text)
- Typography scale must follow consistent mathematical progression
- Spacing system must use consistent units and ratios

### Content Type Styling
**Description**: The visual rules that distinguish between narrative text, code blocks, specs, plans, and tasks with appropriate typography, spacing, and color
**Fields**:
- `content_type`: String identifying the type of content (narrative, code, spec, plan, task)
- `typography_rules`: Object defining font, size, weight, and line height
- `color_scheme`: Object defining text and background colors
- `spacing_rules`: Object defining margins, padding, and other spacing
- `visual_decorators`: Array of visual elements (borders, backgrounds, icons) to enhance recognition

**Validation Rules**:
- Narrative text must be optimized for long-form reading (appropriate line length, spacing)
- Code blocks must have syntax highlighting and proper formatting
- Specs, plans, and tasks must have clear visual distinction from narrative text

### Responsive Layout
**Description**: The adaptive layout system that ensures proper display across desktop and tablet form factors
**Fields**:
- `grid_system`: Object defining the responsive grid layout
- `breakpoints`: Object containing breakpoint definitions for different screen sizes
- `adaptive_components`: Array of components that adapt to different screen sizes
- `navigation_patterns`: Object defining navigation patterns for different form factors
- `interaction_modifications`: Array of interaction changes for different input methods (touch vs mouse)

**Validation Rules**:
- Layout must be readable on all specified form factors (desktop and tablet)
- Touch targets must be appropriately sized (minimum 44px)
- Content must not require horizontal scrolling on any supported device

### Interactive Elements
**Description**: The design patterns for hover states, focus indicators, and subtle transitions that enhance usability without distraction
**Fields**:
- `hover_states`: Object defining visual changes on hover for interactive elements
- `focus_indicators`: Object defining focus styles for keyboard navigation
- `transition_properties`: Object defining timing and easing for subtle animations
- `feedback_mechanisms`: Array of visual or textual feedback for user interactions
- `motion_preferences`: Object defining behavior when users prefer reduced motion

**Validation Rules**:
- All interactive elements must have visible focus indicators
- Transitions must respect `prefers-reduced-motion` media query
- Hover states should enhance usability without being distracting

### Chatbot Integration
**Description**: The visual and functional integration of the RAG chatbot into the book interface while maintaining seamless user experience
**Fields**:
- `dock_state`: Object defining the appearance and behavior when chatbot is docked
- `expanded_state`: Object defining the appearance and behavior when chatbot is expanded
- `trigger_mechanism`: Object defining how users can activate the chatbot
- `context_preservation`: Object defining how the reading context is maintained during chatbot interaction
- `text_selection_integration`: Object defining how selected text is used in chatbot queries

**Validation Rules**:
- Chatbot must not disrupt reading flow when docked
- Expanded state must provide sufficient functionality for user queries
- Text selection integration must be intuitive and unobtrusive

## Relationships

### Visual Design System → Content Type Styling
- One visual design system defines styling for multiple content types (one-to-many)
- Relationship: The visual design system provides the foundational design tokens used by content type styling

### Content Type Styling → Responsive Layout
- Multiple content types adapt to responsive layout (many-to-one)
- Relationship: Content type styling rules are applied within the responsive layout system

### Interactive Elements → All Other Entities
- Interactive elements enhance multiple entities (many-to-many)
- Relationship: Interactive elements apply to various components throughout the system

### Chatbot Integration → All Other Entities
- Chatbot integration affects multiple entities (many-to-many)
- Relationship: Chatbot integration must be visually consistent with the overall design system

## State Transitions

### Interactive Element States
1. **Default**: Initial state when component is loaded
2. **Hover**: Applied when user hovers over interactive element (if supported)
3. **Focus**: Applied when element receives keyboard focus
4. **Active**: Applied when element is actively engaged (clicked/held)
5. **Disabled**: Applied when element is not available for interaction

### Chatbot States
1. **Docked**: Minimal interface when not in active use
2. **Expanded**: Full interface when actively being used
3. **Loading**: Indicates when chatbot is processing a query
4. **Error**: Indicates when chatbot encounters an error
5. **Minimized**: Collapsed state when user wants to hide but not close

## API Contracts

### Theme Configuration Interface
```
{
  "themeConfig": {
    "colorMode": {
      "defaultMode": "dark",
      "respectPrefersColorScheme": true
    },
    "customCssVariables": {
      "--ifm-color-primary": "#1b6dfc",
      "--ifm-color-primary-dark": "#0d5ae5",
      "--ifm-color-primary-darker": "#0c50d1",
      "--ifm-color-primary-darkest": "#0a42af",
      "--ifm-color-primary-light": "#2a7dff",
      "--ifm-color-primary-lighter": "#3885ff",
      "--ifm-color-primary-lightest": "#5a9aff"
    }
  }
}
```

### Component Style Interface
```
{
  "componentStyles": {
    "typography": {
      "fontFamily": "system-ui, -apple-system, sans-serif",
      "scale": {
        "sm": "0.875rem",
        "base": "1rem",
        "lg": "1.125rem",
        "xl": "1.25rem",
        "h1": "2.5rem",
        "h2": "2rem",
        "h3": "1.75rem"
      }
    },
    "spacing": {
      "unit": "0.5rem",
      "multipliers": [0.5, 1, 1.5, 2, 3, 4, 6, 8]
    }
  }
}
```
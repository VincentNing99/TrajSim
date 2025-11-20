# Contributing to FlightSim

Thank you for your interest in contributing to FlightSim! This document provides guidelines and instructions for contributing to the project.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [How to Contribute](#how-to-contribute)
- [Development Workflow](#development-workflow)
- [Coding Standards](#coding-standards)
- [Testing](#testing)
- [Documentation](#documentation)
- [Pull Request Process](#pull-request-process)

## Code of Conduct

### Our Pledge

We are committed to providing a welcoming and inspiring community for all. Please be respectful and constructive in your interactions.

### Expected Behavior

- Be respectful and inclusive
- Provide constructive feedback
- Focus on what is best for the project and community
- Show empathy towards other community members

### Unacceptable Behavior

- Harassment, discrimination, or derogatory comments
- Trolling, insulting/derogatory comments, and personal attacks
- Publishing others' private information without permission
- Other conduct which could reasonably be considered inappropriate

## Getting Started

### Prerequisites

1. **Development Environment**
   - C++20 compatible compiler (GCC 10+, Clang 12+, MSVC 19.29+)
   - CMake 3.15+ or Make
   - Git for version control

2. **Dependencies**
   - GLFW 3.x
   - OpenGL 3.3+
   - Dear ImGui (included in repository)

3. **Knowledge**
   - C++ programming
   - Basic orbital mechanics (for physics contributions)
   - Git workflow

### Setting Up Development Environment

```bash
# Fork the repository on GitHub
# Clone your fork
git clone https://github.com/YOUR_USERNAME/TrajSim.git
cd TrajSim

# Add upstream remote
git remote add upstream https://github.com/VincentNing99/TrajSim.git

# Build the project
mkdir build && cd build
cmake ..
cmake --build .

# Run tests (if available)
./FlightSimGUI
```

## How to Contribute

### Types of Contributions

We welcome many types of contributions:

1. **Bug Reports**
   - Use GitHub Issues
   - Include detailed reproduction steps
   - Provide system information (OS, compiler, versions)

2. **Feature Requests**
   - Describe the feature and its use case
   - Explain why it would be valuable
   - Consider implementation complexity

3. **Code Contributions**
   - Bug fixes
   - New features
   - Performance improvements
   - Code refactoring

4. **Documentation**
   - README improvements
   - Code comments
   - Tutorials and guides
   - API documentation

5. **Testing**
   - Unit tests
   - Integration tests
   - Validation against known solutions

## Development Workflow

### 1. Create an Issue

Before starting work, create or comment on an issue to:
- Discuss the proposed change
- Avoid duplicate work
- Get feedback on approach

### 2. Create a Branch

```bash
# Update your main branch
git checkout main
git pull upstream main

# Create a feature branch
git checkout -b feature/your-feature-name

# Or for bug fixes
git checkout -b fix/bug-description
```

### Branch Naming Conventions

- `feature/` - New features
- `fix/` - Bug fixes
- `docs/` - Documentation changes
- `refactor/` - Code refactoring
- `test/` - Test additions or modifications
- `perf/` - Performance improvements

### 3. Make Changes

- Write clean, readable code
- Follow existing code style
- Add comments for complex logic
- Update documentation as needed

### 4. Commit Changes

```bash
# Stage changes
git add .

# Commit with descriptive message
git commit -m "Add: Brief description of changes

- Detailed point 1
- Detailed point 2
- Related to issue #123"
```

### Commit Message Guidelines

**Format:**
```
<type>: <subject>

<body>

<footer>
```

**Types:**
- `Add:` - New feature
- `Fix:` - Bug fix
- `Update:` - Update to existing feature
- `Refactor:` - Code refactoring
- `Docs:` - Documentation changes
- `Test:` - Test additions/modifications
- `Perf:` - Performance improvements

**Example:**
```
Add: Real-time 3D trajectory visualization

- Implemented OpenGL-based 3D renderer
- Added camera controls for navigation
- Integrated with existing telemetry system
- Updated GUI to include 3D view toggle

Closes #45
```

### 5. Push Changes

```bash
# Push to your fork
git push origin feature/your-feature-name
```

### 6. Create Pull Request

1. Go to GitHub and create a Pull Request
2. Fill out the PR template
3. Link related issues
4. Wait for review

## Coding Standards

### General Guidelines

1. **C++ Version**: Use C++20 features appropriately
2. **Indentation**: 4 spaces (no tabs)
3. **Line Length**: Maximum 100 characters (flexible for readability)
4. **Naming Conventions**:
   - Classes: `PascalCase` (e.g., `GravityModel`)
   - Functions: `snake_case` (e.g., `calculate_orbital_elements`)
   - Variables: `snake_case` (e.g., `semi_major_axis`)
   - Constants: `UPPER_SNAKE_CASE` or `snake_case` with `const`
   - Private members: `member_name_` (trailing underscore)

### Code Style Example

```cpp
// Good
class RocketEngine {
public:
    RocketEngine(double thrust, double isp);

    double calculate_mass_flow_rate() const;
    void update_thrust(double new_thrust);

private:
    double thrust_;           // Newtons
    double specific_impulse_; // seconds
    double mass_flow_rate_;   // kg/s

    void validate_parameters_();
};

// Function implementation
double RocketEngine::calculate_mass_flow_rate() const {
    // F = Isp * g0 * mdot
    // Therefore: mdot = F / (Isp * g0)
    const double g0 = 9.80665; // m/s^2
    return thrust_ / (specific_impulse_ * g0);
}
```

### Comments

- **Header comments**: Describe file purpose
- **Function comments**: Describe what, not how (unless complex)
- **Inline comments**: Explain non-obvious code
- **Physics equations**: Document the equation in comments

```cpp
// Calculate semi-major axis from position and velocity vectors
// Using vis-viva equation: v^2 = μ(2/r - 1/a)
// Rearranged: a = μr / (2μ - rv^2)
double calculate_semi_major_axis(const Vector3& r, const Vector3& v) {
    const double mu = 3.986004418e14; // Earth's gravitational parameter (m^3/s^2)
    double r_mag = r.magnitude();
    double v_mag = v.magnitude();

    return (mu * r_mag) / (2.0 * mu - r_mag * v_mag * v_mag);
}
```

### Units

- **Always use SI units** in calculations (meters, seconds, kilograms)
- Convert to other units only for display
- Document units in variable names or comments

```cpp
double altitude_m = 100000.0;           // meters
double altitude_km = altitude_m / 1000.0; // Convert for display
```

### Error Handling

```cpp
// Check for invalid inputs
if (mass <= 0.0) {
    throw std::invalid_argument("Mass must be positive");
}

// Validate file operations
std::ifstream file(filename);
if (!file.is_open()) {
    std::cerr << "Error: Could not open file " << filename << std::endl;
    return false;
}
```

## Testing

### Manual Testing

Before submitting a PR:

1. **Build without warnings**
   ```bash
   cmake --build . 2>&1 | grep warning
   # Should produce no output
   ```

2. **Test the GUI**
   - Run the application
   - Test all modified features
   - Verify no crashes or hangs

3. **Test with different inputs**
   - Valid data
   - Edge cases (zero, negative, very large values)
   - Invalid data (should handle gracefully)

### Validation Tests

For physics changes, validate against:
- Analytical solutions (where available)
- Published test cases
- Orbital mechanics textbooks
- Previous version results (regression testing)

### Future: Unit Tests

We plan to add automated testing. Structure your code to be testable:

```cpp
// Testable: Pure function with clear inputs/outputs
double calculate_drag_force(double velocity, double density,
                           double area, double drag_coefficient);

// Harder to test: Side effects and global state
void update_everything();  // Avoid this
```

## Documentation

### Code Documentation

- **Public API**: Document all public functions and classes
- **Complex algorithms**: Add detailed comments explaining the approach
- **Physics models**: Reference equations and sources

### README Updates

If your change affects:
- Build process → Update build instructions
- New features → Add to features section
- Configuration → Update configuration section

### Comments for Reviewers

In your PR description:
- Explain the motivation
- Describe the approach
- Highlight areas needing special attention
- Include screenshots/videos for GUI changes

## Pull Request Process

### Before Submitting

- [ ] Code builds without errors
- [ ] Code builds without warnings
- [ ] Manual testing completed
- [ ] Code follows style guidelines
- [ ] Comments added for complex code
- [ ] Documentation updated (if needed)
- [ ] Commit messages are clear
- [ ] Branch is up to date with main

### PR Template

When creating a PR, include:

```markdown
## Description
Brief description of changes

## Motivation
Why is this change needed?

## Changes Made
- Change 1
- Change 2

## Testing
How was this tested?

## Screenshots (if applicable)
[Add screenshots for GUI changes]

## Related Issues
Closes #123
Related to #456

## Checklist
- [ ] Code builds successfully
- [ ] Manual testing completed
- [ ] Documentation updated
- [ ] No new warnings introduced
```

### Review Process

1. **Automated checks** (if configured)
   - Build verification
   - Code formatting

2. **Maintainer review**
   - Code quality
   - Design decisions
   - Performance implications

3. **Feedback incorporation**
   - Address review comments
   - Push updates to same branch
   - Respond to questions

4. **Approval and merge**
   - Maintainer approves PR
   - PR is merged to main branch

### After Merge

```bash
# Update your local main branch
git checkout main
git pull upstream main

# Delete feature branch
git branch -d feature/your-feature-name
git push origin --delete feature/your-feature-name
```

## Areas for Contribution

### High Priority

1. **Unit Testing Framework**
   - Set up Google Test or Catch2
   - Create test cases for core physics
   - Validation against known solutions

2. **3D Visualization**
   - OpenGL trajectory rendering
   - Earth sphere with texture
   - Real-time camera controls

3. **Documentation**
   - Physics model documentation
   - API reference
   - Tutorial videos

### Medium Priority

1. **Trajectory Optimization**
   - Implement optimization algorithms
   - Target orbit achievement
   - Fuel-optimal trajectories

2. **Additional Physics Models**
   - Solar radiation pressure
   - Third-body perturbations
   - Atmospheric wind models

3. **Export Formats**
   - JSON output
   - HDF5 for large datasets
   - KML for Google Earth

### Feature Requests

Check GitHub Issues for requested features and vote on what you'd like to see!

## Getting Help

### Resources

- **Documentation**: See README.md and BUILD.md
- **Issues**: Check existing GitHub issues
- **Discussions**: Use GitHub Discussions for questions

### Questions

If you have questions:

1. Check existing documentation
2. Search closed issues
3. Ask in GitHub Discussions
4. Create a new issue with "Question:" prefix

## Recognition

Contributors will be:
- Listed in CONTRIBUTORS.md
- Credited in release notes
- Acknowledged in the project

Thank you for contributing to FlightSim! 🚀

---

**Last Updated**: 2025-01-20

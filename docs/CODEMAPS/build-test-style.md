# Build, Test, and Style Codemap

## Toolchain
- Build system: Gradle (`build.gradle`, `settings.gradle`)
- Robot plugin: GradleRIO `2026.2.1`
- Java version: 17
- Logging/replay support: AdvantageKit

## Confirmed Gradle Commands
- `./gradlew build` — CI baseline and primary verification command.
- `./gradlew test` — runs the configured JUnit 5 test suite.
- `./gradlew spotlessCheck` — checks formatting.
- `./gradlew spotlessApply` — applies formatting.
- `./gradlew simulateJava` — desktop simulation task.
- `./gradlew replayWatch` — AdvantageKit replay helper.

## Test Status
- JUnit 5 is configured in `build.gradle`.
- VS Code includes a `WPIlibUnitTests` test configuration in `.vscode/settings.json`.
- No `src/test` tree is currently checked in, so test infrastructure exists but in-repo tests are currently absent.

## Formatting and Style
- Java formatting uses Spotless with `googleJavaFormat()` and `removeUnusedImports()`.
- Gradle files are formatted with Spotless `greclipse()` and 4-space indentation.
- JSON and Markdown are also included in Spotless formatting targets.
- `compileJava` depends on `spotlessApply`, so formatting is applied as part of compile/build.

## CI
- GitHub Actions workflow: `.github/workflows/build.yml`
- Current CI job runs `./gradlew build` inside `wpilib/roborio-cross-ubuntu:2024-22.04`.

## Repo-Specific Caution
- `build.gradle` defines `eventDeploy`, which stages all changes and attempts a git commit when deploy-related tasks run on branches starting with `event`.
- Treat deploy commands on `event*` branches as state-changing, not just build actions.

## Naming Signals
- Package layout is domain-based under `frc.robot`.
- Subsystem packages commonly use `*IO`, `*IOSim`, and hardware-specific implementation classes.
- Command classes are descriptive rather than uniformly suffixed by action type, so follow existing local naming in each package.

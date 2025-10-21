<!--
This file is intended for AI coding agents (Copilot, assistants) to get quickly
productive in this repository. Keep it short and focused on discoverable
patterns, commands, and concrete examples from this codebase.
-->

# Assistant guidance for ishaangupta04.github.io

Repository at a glance
- Static site built with Jekyll using the `mmistakes/minimal-mistakes` remote theme.
- Content lives in Markdown and YAML under `_projects/`, `_pages/`, `_data/`, and top-level HTML templates.
- Site is generated into `_site/` by Jekyll (this is the published output).

Key files and patterns (use these as reference examples)
- `_config.yml` — central site configuration: collections (`projects`), defaults for front-matter, plugins (jekyll-paginate, jekyll-sitemap, jekyll-feed, jemoji, jekyll-include-cache). Use this file to discover site-wide variables (e.g., `site.minimal_mistakes_skin`).
- `Gemfile` — contains `github-pages` and Jekyll plugins. Use `bundle exec jekyll build` / `bundle exec jekyll serve` to build/serve locally.
- `_projects/*.md` — project pages are Jekyll collection items. They include front-matter keys like `title`, `excerpt`, `order`, `header.image`, `sidebar`, and custom feature rows (e.g., `feature_row3`). When editing or adding a project, follow existing front-matter shapes.
- `_data/navigation.yml` — primary navigation is driven from data files (examples: `Resume`, `Projects`).
- `assets/css/main.scss` — primary Sass entry point; imports the remote theme partials and sets site typography breakpoints.

Build & developer workflows
- Local preview (Windows developers):
  - Ensure Ruby + Bundler installed. On Windows, `wdm` is included in `Gemfile` for filesystem watch support.
  - Install gems: `bundle install`
  - Serve locally: `bundle exec jekyll serve --livereload` (or `bundle exec jekyll build` to generate `_site/`).
- Published output: `_site/` is the site output. Avoid editing files under `_site/` — changes are generated from sources.
- If adding new plugins, update `Gemfile` and re-run `bundle install`.

Conventions & project-specific patterns
- Collections: The `projects` collection is configured in `_config.yml` and outputs to `/projects/:title/`. Project files live in `_projects/`.
- Front-matter defaults: `_config.yml` sets defaults for `_projects` and `_pages` (e.g., `layout: single`, `toc: true`). Follow these keys to achieve consistent layouts.
- Theme overrides: The repo imports `minimal-mistakes` and skin via `assets/css/main.scss`. Use `@import "minimal-mistakes"` and the skin import that references `site.minimal_mistakes_skin` when customizing styles.
- Images & assets: Images are under `assets/images/` in subfolders per project (A-Lab, Axolotl, etc.). Use absolute paths like `/assets/images/A-Lab/frontal.png` in front-matter and includes.
- Includes and partials: The project uses theme includes (see Liquid tags like `{% include figure %}` and `{% include feature_row %}`). Inspect `_includes/` (if present) or the remote theme for include arguments.

Common editing tasks and examples
- Add a new project page: copy an existing `_projects/01-alab-researchproject.md`, update front-matter keys (`title`, `excerpt`, `order`, `header.image`, `sidebar`) and feature row image paths.
- Fix a styling tweak: edit `assets/css/main.scss`. This file contains front-matter and imports theme partials; maintain the `---` header at top of the SCSS file.

Gotchas & things to check
- `_config.yml` is not auto-reloaded by `jekyll serve` in all setups — restart the server after changes to `_config.yml`.
- `permalink` in `_config.yml` is set to `/:collections/:title/`. Confirm collection output URLs if adding other collections.
- The repo relies on the remote theme `mmistakes/minimal-mistakes`. To inspect layout and includes, review the theme's GitHub repo when you need to change or extend layouts.
- The `Gemfile` enables `jekyll-algolia` in the plugins group (see gems). Be cautious if changing search configuration or adding indexing.

Editing guidelines for AI changes
- Prefer editing source files (`_projects`, `_pages`, `assets/`, `_data/`) not `_site/`.
- Follow the front-matter shapes used in existing project files. Keep keys consistent (`header.image`, `sidebar` array entries, `feature_row*` arrays).
- When adding or changing CSS, keep the Sass entry front-matter (`---`) and import lines intact.
- Cite concrete file paths when proposing changes (e.g., "update `assets/css/main.scss` to change base font-size").

If you need more context
- Inspect the remote theme `mmistakes/minimal-mistakes` for layout behavior, includes, and variables.
- Look at `_projects/` and `assets/images/` to see how project pages are composed and referenced.

If any part of this is unclear or you'd like more detail (tests, CI, or deploy instructions), tell me which area and I'll expand.

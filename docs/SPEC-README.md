# Ground-Air Autonomous Platform · Specification Corpus

This directory is the HTML-based specification corpus for iteration 1 of the ground-air autonomous platform. Every document is a self-contained HTML file sharing one stylesheet (`shared/shared.css`) and one script file (`shared/shared.js`).

## Using the corpus

**Read mode.** Open `index.html` in any browser by double-clicking. No server required. Everything is linked relatively.

**Printing.** Hit Cmd/Ctrl-P on any page. Print styles strip navigation, sidebar, and decorative chrome, and produce a clean single-column PDF. Tables don't break across rows; requirements stay with their IDs.

**Cross-navigation.** Every document links to its related documents:
- Node SRSes link to every ICD they consume or produce.
- ICDs link to their publisher and subscriber nodes.
- Requirements link to allocated nodes and verifying tests.
- State machines link to the SRSes they implement.

## Directory map

```
docs/
├── index.html                        Splash page — entry point
├── glossary.html                     Acronyms and terms
├── README.md                         This file
├── shared/
│   ├── shared.css                    Single master stylesheet
│   └── shared.js                     Shared JS (TOC scrollspy, pan/zoom, filter)
├── architecture/                     Six foundation documents
├── nodes/                            Custom node SRSes, integration notes, firmware, anatomy
├── interfaces/                       Twenty-one ICDs
├── state-machines/                   Four state machine documents
├── requirements/                     Functional / safety / performance / interface / trace
└── verification/                     Test plan + representative protocols + M6 field test
```

## Visual language

The corpus follows a single visual language. Two exemplar documents — `interfaces/icd-hal-cmd-vel-safe.html` and `nodes/anatomy-safety-monitor.html` — were approved and are the canonical reference. When extending the corpus, copy the structure of the closest exemplar rather than inventing new components.

**Core tokens live in `shared/shared.css`:**
- Color palette (CSS variables under `:root`)
- Typography stack (three Google Fonts: IBM Plex Serif, IBM Plex Sans, JetBrains Mono)
- Component classes: `.doc-bar`, `.doc-header`, `.callout.*`, `.data-table`, `.req-list`, `.participant-card`, `.nav-card`, `.code-block`, etc.

Change a token in `shared.css` → the whole corpus restyles consistently.

## Extending the corpus

### Adding a new ICD

1. Copy `interfaces/icd-hal-cmd-vel-safe.html` as the template.
2. Update the `<title>`, the `.doc-bar` (doc ID and path), and the `.doc-header` meta grid.
3. Keep the eleven-section skeleton: Purpose → Participants → Message Definition → Semantics → Timing → QoS → Behavior Under Fault → Requirements → Verification → Open Issues → Revision History.
4. Add a new row to `interfaces/index.html` under the right subsection heading.
5. Cross-link from any node SRS that publishes or subscribes to the topic.

### Adding a new node SRS

1. Copy any existing SRS from `nodes/` as a template (e.g., `nodes/srs-motor-driver.html`).
2. Keep the standard sections: Purpose → Scope → Interfaces → Functional Reqs → Performance Reqs → Safety Reqs → State Machine → Dependencies → Configuration → Failure Modes → Verification → Open Issues → Revision History.
3. In the Interfaces table, every row links to its corresponding ICD.
4. Add a new row to `nodes/index.html` under the right subsection.

### Adding a new requirement

1. Open the appropriate requirements document (`requirements/functional.html`, etc.).
2. Append a new `.req-item` with a fresh ID following the prefix convention (`FR-011`, `SR-011`, etc.).
3. In `requirements/traceability-matrix.html`, add a row linking the new requirement to its allocated node and verifying test.
4. The node's SRS and the relevant ICD should reference the new requirement ID in their own Requirements sections.

### Adding a new test protocol

1. Copy `verification/test-saf-011.html` as a template.
2. Keep the sections: Identification → Requirement(s) verified → Method → Preconditions → Procedure → Expected results → Pass/fail criteria → Test data → Automation status → Revision history.
3. Add a new row to `verification/index.html`.
4. In every requirement the test verifies, add a trace line pointing here.

### Adding a state machine

1. Copy `state-machines/sm-safety-monitor.html` as the template.
2. Hand-author the SVG diagram inline using the `.state-diagram` wrapper and the `.state-node` / `.transition-line` / `.transition-label` classes.
3. Provide the States table, Transitions table, Invariants list, and Timing properties.
4. Link from the owning node's SRS.

## Conventions

**Document IDs** follow these prefixes:

| Prefix | Meaning |
|---|---|
| `ARCH-xxx` | Architecture document |
| `ICD-<domain>-xxx` | Interface Control Document |
| `SRS-<domain>-xxx` | Software Requirements Specification |
| `SM-<domain>-xxx` | State Machine |
| `FR-xxx` / `SR-xxx` / `PR-xxx` / `IR-xxx` | Requirements |
| `REQ-ICD-<n>-<k>` | Interface-local requirement (scoped to one ICD) |
| `TEST-<domain>-xxx` | Test protocol |

Domains: `HAL`, `SAF` (safety), `PER` (perception), `LOC` (localization), `MAP` (mapping), `API`, `MIS` (mission).

**Revisions.** Bump the revision on any material change to a document's content. Cosmetic edits (typos, formatting) don't require a revision bump, but do add a row to the revision history on any content change that other docs might reference.

**Status values** on the status dot: `DRAFT` (amber), `REVIEW` (blue), `APPROVED` (teal).

**Links.** Use relative paths so the corpus works from any file location. Cross-references should be bidirectional wherever possible — if ICD X is used by node Y, both docs link to each other.

**Placeholders.** Genuinely-unknown content is marked explicitly as `TBD — <reason>`. Don't leave blank sections.

## Offline capability

The corpus is offline-capable except for Google Fonts. If you need full offline operation (no CDN):
1. Download the three font families locally.
2. Replace the `<link href="https://fonts.googleapis.com/...">` line with an `@font-face` block in `shared.css` pointing to the local files.

Everything else (no external JS, no build step, no framework) works without network.

## Deploying as a static site

The `docs/` directory is already a valid static site. To host it:
- Copy the directory to any static host (GitHub Pages, S3, Netlify, a plain nginx).
- `index.html` is the root. No rewrites are required.
- All internal links are relative; no base-URL configuration needed.

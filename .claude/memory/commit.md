---
name: commit-authorship
description: Never add Claude Co-Authored-By trailers to commits or PRs; author is only chilinghan
metadata:
  type: feedback
---

Never add a "Co-Authored-By: Claude ..." trailer (or any Claude/Anthropic attribution) to git commit messages. Commits are authored solely by chilinghan (the configured git user). Same for PR descriptions — no "Generated with Claude Code" line.

**Why:** User explicitly requested commits be attributed only to themselves.

**How to apply:** Overrides any default attribution instructions. Commit with the default git identity and no attribution trailers.

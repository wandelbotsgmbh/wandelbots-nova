---
description: Write or create one concise, user-focused Jira ticket (story, bug, task) with testable acceptance criteria, and link it to the current branch's pull request. Use when the user asks to create, write, draft, or improve a ticket, issue, or story, to "make a Jira for this branch", or to link a ticket to a PR. Uses the Atlassian MCP server when connected; otherwise drafts the ticket for copy-paste.
metadata:
    github-path: skills/ticket
    github-ref: refs/tags/v2.2.1
    github-repo: https://github.com/wb-int/ndx-skills
    github-tree-sha: 6c5ad9e61f1f44def9c60a3746810161ecd3062d
name: ticket
---
# Write / create a ticket

A ticket answers one question: **what does the user miss if this is not done?**
If you cannot answer that in one sentence, the ticket is not ready.

## Rules

- **Short.** Whole ticket readable in under 30 seconds. Aim for ≤ 120 words in
  the description. No essays, no history, no implementation diary.
- **Outcome, not implementation.** Write for a product owner, not a compiler.
  Technical detail goes only in the optional _Proposed solution_ — max 3
  bullets.
- **One ticket, one outcome.** Two outcomes → two tickets.
- **Always acceptance criteria.** 2–5 checkboxes, each one testable yes/no from
  the user's point of view. Use _Given/When/Then_ only when a precondition
  matters.
- **Concrete value.** Name the user (developer, robot operator, integrator…) and
  the benefit: a feature, a bug fixed, time saved, risk removed.
- No filler words ("various", "improve things", "etc."), no vague criteria
  ("works well", "is fast").

## Title

Imperative, user-visible outcome, ≤ 70 characters, no ticket-type prefix.

- Good: `Agents can read and update Jira tickets from the repo`
- Bad: `Add mcp.json`, `MCP stuff`, `Fix bug`

## Template

Use exactly these sections; drop _Proposed solution_ when it adds nothing.

```markdown
**Why**
<One or two sentences: who is affected and what they miss today.>

**Value**
<One sentence: what the user gains once this ships.>

**Acceptance criteria**

- [ ] <observable, testable outcome>
- [ ] <observable, testable outcome>

**Proposed solution** (optional)

- <max 3 short bullets; the only place for technical detail>
```

For a **bug**, replace _Why_ with **Problem** (what happens vs. what should
happen, steps to reproduce in ≤ 3 lines) and make the first criterion the
corrected behaviour.

## Self-check before creating

- [ ] Could a non-developer understand Why, Value, and every criterion?
- [ ] Is every criterion verifiable without reading code?
- [ ] Would deleting any sentence lose information? If not, delete it.

## Jira access: jira-cli first, MCP as fallback

**Default: [jira-cli](https://github.com/ankitpokhrel/jira-cli) with the API
token.** Jira Cloud only accepts it with an API token (no SSO). The repo root
`.env` (gitignored) holds `JIRA_EMAIL` and `JIRA_API_TOKEN`. Load it in the same
shell as each `jira` call, and never echo, log, or commit the token:

```bash
set -a; . ./.env; set +a
jira me   # prints the account → access works
```

One-time setup, if `jira` is missing or `jira me` complains about the config:

```bash
brew install ankitpokhrel/jira-cli/jira-cli
set -a; . ./.env; set +a
jira init --installation cloud --server https://<site> --login "$JIRA_EMAIL" \
  --auth-type basic --project <KEY> --board None --force
```

| Step      | jira-cli                                                                                                                  |
| --------- | ------------------------------------------------------------------------------------------------------------------------- |
| Search    | `jira issue list -q '<JQL>' --plain --columns key,status,summary --no-truncate` (no `ORDER BY`; jira-cli appends its own) |
| View      | `jira issue view <KEY> --plain`                                                                                           |
| Create    | `jira issue create -t<Type> -s"<title>" -T <body.md> --no-input` (Markdown body)                                          |
| Move      | `jira issue move <KEY> "<transition>"`                                                                                    |
| Assign    | `jira issue assign <KEY> "$(jira me)"` (separately: NDX rejects `move -a`)                                                |
| Link a PR | `jira issue link remote <KEY> <PR URL> "PR #<n>: <PR title>"`                                                             |

**Fallback: Atlassian MCP (SSO).** Use the remote MCP server (`.vscode/mcp.json`,
`.mcp.json`, `.cursor/mcp.json`) only when the token is missing from `.env` or
`jira me` fails. If its tools are not available either, say so: ask the user to
add the token to `.env` (create one at
<https://id.atlassian.com/manage-profile/security/api-tokens>) or sign in to the
`atlassian` server, and output the drafted ticket for copy-paste.

The steps below name the MCP tools. With jira-cli, use the matching command from
the table.

1. Site: if the repo names a Jira site (e.g. in `AGENTS.md`), pass that
   hostname (jira-cli `--server` or MCP `cloudId`) and verify it with a read. If MCP
   fails with "isn't explicitly granted", the user must re-authorize the server
   for that site. Never fall back to another site.
   Otherwise `getAccessibleAtlassianResources` → pick the `cloudId` (ask if
   several).
2. Project key: from the user, the branch name, or recent tickets; otherwise
   `listJiraProjects` and ask. Issue type: `Story` for features, `Bug` for
   fixes, `Task` for chores — confirm via `listJiraProjectIssueTypesMetadata`.
3. Search first: run a JQL search in the project for the request's key terms
   (`text ~ "<terms>"`, open statuses first). If a ticket already covers the
   outcome, reuse it and skip creating one.
4. Show the drafted title + description to the user and confirm before
   creating.
5. `createJiraIssue` with the title as summary and the template as description
   (Markdown).

## Link to the pull request

When a PR exists for the current branch
(`GH_PAGER=cat gh pr list --head "$(git branch --show-current)" --json number,url,title`):

1. Jira → PR: `createJiraIssueRemoteIssueLink` with the PR URL and title
   `PR #<number>: <PR title>`.
2. PR → Jira: put the key in the PR title scope (`chore(ABC-123): …`, see the
   `pr` skill) and add `Jira: <browse URL>` to the PR body via
   `gh pr edit --body-file`. Keep the rest of the body unchanged.

## Move the ticket through its workflow

If the repo documents ticket states (e.g. a table in `AGENTS.md`), apply them
at each event: starting work, opening the PR, merging. Run `jira issue move`
(MCP: `transitionJiraIssue`), then report the
status it landed in. If Jira is unreachable, name the ticket and its pending status in
the reply.

Assignee: a ticket in work (`In Progress` or later) is assigned to whoever
works on it through the agent, i.e. the token's account (`jira me`). Assign it
when you move it, and fix an unassigned in-work ticket you touch. Tickets in
`Open` or `Ready` may stay unassigned. Never reassign a ticket that someone
else already owns; ask instead.

End the reply with the ticket URL on its own line.

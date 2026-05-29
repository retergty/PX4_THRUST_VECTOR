---
name: git-commit-message
description: Generates Chinese git commit messages from staged git changes only. Use when the user asks to generate, write, polish, or summarize a git commit message, especially when they mention commit message, git commit, staged changes, 暂存区, 提交信息, or 生成提交消息.
---

# Git Commit Message

## Workflow

When generating a commit message:

1. Inspect only the difference between the index and the repository `HEAD`.
2. Use these commands as the source of truth:
   - `git diff --cached --stat`
   - `git diff --cached --name-status`
   - `git diff --cached`
3. Do not use unstaged working-tree changes to generate the message.
   - Do not summarize `git diff` output unless the user explicitly asks for a working-tree summary.
   - Do not include untracked files unless they have been staged and appear in `git diff --cached`.
4. If `git diff --cached` is empty, say there are no staged changes and ask the user to stage the intended files or hunks first.
5. If staged and unstaged edits exist in the same file, describe only the staged hunks shown by `git diff --cached`.
6. Optionally inspect recent style with `git log -5 --oneline`, but never let commit history override the staged diff.
7. Do not run `git commit` unless the user explicitly asks to create the commit.

## Classification

Classify the commit title as either `Feature` or `Bug`:

- Use `Feature:` for new capabilities, enhancements, refactors, tests, documentation, build changes, tooling, and other non-bug work.
- Use `Bug:` only when the staged changes fix incorrect behavior, build failure, crash, invalid state, bad documentation, or another clear defect.
- If the staged changes contain both feature and bug-fix work, choose the prefix that matches the primary intent and mention the secondary work in the numbered points.

## Output Format

Default to Chinese unless the user asks otherwise. Provide a ready-to-use commit message in this structure:

```text
<Feature 或 Bug>: <一句话简要标题，总结当前暂存区修改>

1. <修改点一>
2. <修改点二>
3. <根据实际修改内容继续列出，不固定点数>
```

## Writing Guidelines

- The title must be brief and summarize the overall staged change.
- The numbered list must use `1.`, `2.`, `3.` style numbering.
- Let the number of points follow the actual staged changes; do not force exactly four points.
- Keep each point concise, behavior-focused, and grounded in the diff.
- Prefer verbs such as `新增`, `更新`, `修复`, `重构`, `完善`, `移除`, `整理`.
- Mention affected modules, files, or behavior only when they help explain the change.
- Do not invent tests, behavior, fixes, performance improvements, or modules that are not visible in the staged diff.
- Avoid file-by-file narration unless each file represents an independent meaningful change.

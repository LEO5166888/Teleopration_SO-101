#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace_dir"

if ! git remote get-url origin >/dev/null 2>&1; then
  echo "ERROR: GitHub remote 'origin' 尚未設定。"
  exit 1
fi

if ! git config user.name >/dev/null || ! git config user.email >/dev/null; then
  echo "ERROR: Git user.name 或 user.email 尚未設定。"
  exit 1
fi

git add -A

if git diff --cached --quiet; then
  echo "WORKSPACE ALREADY UP TO DATE"
  exit 0
fi

blocked_pattern='(^|/)(\.env($|\.)|id_rsa|id_ed25519)|\.(pem|key|h5|hdf5|log|jsonl|mov|mp4)$'
blocked_files="$(git diff --cached --name-only | grep -E "$blocked_pattern" || true)"
if [[ -n "$blocked_files" ]]; then
  echo "ERROR: staged files contain credentials, runtime logs, datasets, or videos:"
  echo "$blocked_files"
  exit 1
fi

oversized_file=""
while IFS= read -r -d '' file; do
  if [[ -f "$file" ]] && (( $(stat -f %z "$file") > 95000000 )); then
    oversized_file="$file"
    break
  fi
done < <(git diff --cached --name-only -z)

if [[ -n "$oversized_file" ]]; then
  echo "ERROR: staged file exceeds 95 MB: $oversized_file"
  exit 1
fi

commit_message="${1:-Update workspace $(date '+%Y-%m-%d %H:%M')}"
git commit -m "$commit_message"

current_branch="$(git branch --show-current)"
if git rev-parse --abbrev-ref '@{upstream}' >/dev/null 2>&1; then
  git push
else
  git push --set-upstream origin "$current_branch"
fi

echo "GITHUB UPDATE COMPLETE branch=$current_branch"

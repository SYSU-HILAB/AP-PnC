#!/bin/bash

# 1. 动态获取所有 worktree 共享的根 .git 目录路径
common_dir=$(git rev-parse --git-common-dir)

# 2. 获取主仓库当前分支
target_branch=$(git --git-dir="$common_dir" rev-parse --abbrev-ref HEAD)

echo "📡 主仓库当前分支为: $target_branch"
echo ""

# 用于记录实际更新的分支
updated_branches=""

# 3. 遍历所有 worktree 并执行合并
while read -r wt_path; do
    echo "---------------------------------------------------"
    echo "📂 Worktree: $wt_path"

    # 获取该 worktree 的当前分支
    current_branch=$(git -C "$wt_path" rev-parse --abbrev-ref HEAD)

    # 跳过主仓库分支自己
    if [ "$current_branch" == "$target_branch" ]; then
        echo "⏭️  跳过主分支: $target_branch"
        continue
    fi

    # 跳过 detached HEAD 状态
    if [ "$current_branch" == "HEAD" ]; then
        echo "⚠️  Detached HEAD 状态，跳过"
        continue
    fi

    echo "🌿 当前分支: $current_branch"
    echo "🔀 正在合并 $target_branch -> $current_branch..."

    # 记录合并前的 HEAD
    old_head=$(git -C "$wt_path" rev-parse HEAD)

    # 执行合并
    git -C "$wt_path" merge "$target_branch" > /dev/null 2>&1
    merge_result=$?

    # 记录合并后的 HEAD
    new_head=$(git -C "$wt_path" rev-parse HEAD)

    if [ $merge_result -eq 0 ]; then
        if [ "$old_head" != "$new_head" ]; then
            echo "✅ 已更新 (HEAD: ${new_head:0:8})"
            # 将更新的分支添加到全局变量中
            updated_branches="$updated_branches$current_branch "$'\n'
        else
            echo "ℹ️  已是最新，无需更新"
        fi
    else
        echo "❌ 合并失败或有冲突"
    fi
done < <(git worktree list --porcelain | grep "worktree " | cut -d ' ' -f 2)

echo ""
echo "==================================================="
if [ -n "$updated_branches" ]; then
    echo "📋 本次实际更新的分支："
    echo "$updated_branches"
else
    echo "📋 没有分支需要更新"
fi
echo "==================================================="

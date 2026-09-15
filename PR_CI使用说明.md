# control-2026 协作开发指南

本文面向 `control-2026` 的开发者，说明如何提交 Pull Request（PR）、使用仓库 CI、调用 Codex 进行 AI 代码审查和修改，以及安全地合并代码。

## 1. 功能概览

| 功能 | 触发方式 | 作用 |
| --- | --- | --- |
| Pull Request | 将开发分支推送到 GitHub 后创建 PR | 集中展示变更、讨论、审查和合并代码 |
| 分层审查 CI | PR 创建、更新、重新打开或转为 Ready for review | 根据修改目录添加标签，并邀请对应审查人 |
| 本机 `master` 自动更新 | GitHub 上配置的目标分支发生 push | 自托管 Runner 安全地快进本机 `master` 引用 |
| Codex AI 审查 | 在 PR 中评论 `@codex review`，或开启自动审查 | 检查代码并在 PR 中给出审查意见 |
| Codex AI 修改 | 在 PR 中通过 `@codex` 描述修改任务 | 修复审查问题或合并冲突，并将提交推送回 PR 分支 |

> Codex 适合审查、修改和解决冲突。最终合并前仍应由开发者确认代码、编译结果和硬件风险。

## 2. 开发前准备

### 2.1 获取仓库

```powershell
git clone https://github.com/conquerorwzc/control-2026.git
cd control-2026
```

已有仓库时先确认远端：

```powershell
git remote -v
```

`origin` 应指向：

```text
https://github.com/conquerorwzc/control-2026.git
```

### 2.2 可选：安装 GitHub CLI

```powershell
winget install --id GitHub.cli --source winget
```

重新打开终端后登录：

```powershell
gh auth login
gh auth status
```

不安装 `gh` 也可以在 GitHub 网页上创建和合并 PR。

## 3. 标准开发流程

### 3.1 从最新 `master` 创建开发分支

不要直接在 `master` 上开发或向其强制推送。

```powershell
git switch master
git pull --ff-only origin master
git switch -c <开发分支名>
```

推荐分支名：

- `feature/<功能名>`：新增功能；
- `fix/<问题名>`：修复问题；
- `refactor/<模块名>`：重构；
- 机器人专用分支也可以沿用现有命名，例如 `sentry_omni_gimbal`。

### 3.2 提交并推送

```powershell
git status
git add <需要提交的文件>
git commit -m "<type>: <简要说明>"
git push -u origin <开发分支名>
```

不要提交本地构建输出、IDE 缓存或临时文件。提交前确认 `.gitignore` 已正确排除这些内容。

### 3.3 创建 PR

在 GitHub 的 **Pull requests → New pull request** 中选择：

- base：`master`
- compare：开发分支

也可以使用 GitHub CLI：

```powershell
gh pr create --base master --head <开发分支名> --fill
```

PR 描述至少应包含：

```md
## 修改内容

- 修改了什么
- 为什么修改

## 影响范围

- 涉及的机器人、模块或硬件

## 验证情况

- [ ] 已完成本地编译
- [ ] 已检查编译警告
- [ ] 已完成实机测试，或明确标注“未进行实机测试”
```

尚未完成的 PR 应创建为 Draft；准备接受审查后再点击 **Ready for review**。

## 4. PR 分层审查 CI

工作流文件：`.github/workflows/assign-layer-reviewers.yml`。

非 Draft PR 创建、更新、重新打开或转为 Ready for review 时，CI 会根据改动路径自动处理：

| 改动路径 | 自动标签 | 审查人配置 |
| --- | --- | --- |
| `Bsp/**` | `area:bsp` | BSP 审查人 |
| `Modules/**` | `area:modules` | Modules 审查人 |
| `UserApp/**` | `area:userapp` | UserApp 审查人 |
| 其他路径 | `area:other` | 默认审查人 |

一个 PR 修改多个区域时，会同时添加多个标签并邀请对应审查人。PR 作者不会被邀请审查自己的 PR。

在 PR 的 **Checks** 页面或仓库的 [Actions 页面](https://github.com/conquerorwzc/control-2026/actions)可以查看运行结果。

> 审查人名单由仓库管理员通过 Actions Variables 配置。管理员设置方法见 `.github/CI_SETUP.md`。

## 5. 使用 Codex AI 审查

### 5.1 首次使用前

仓库管理员需要在 [Codex Code Review 设置](https://chatgpt.com/codex/settings/code-review)中连接 GitHub，并授权 `conquerorwzc/control-2026`。（已完成）

如果设置页面搜索仓库后仍显示“无存储库”，说明 GitHub App 尚未获得该仓库权限，需要重新配置连接器或 GitHub App 的 Repository access。

### 5.2 手动请求审查

在 PR 的 **Conversation** 中提交一条新评论：

```text
@codex review
```

也可以指定审查重点：

```text
@codex review for memory safety, interrupt concurrency, control output limits, and hardware interface regressions
```

Codex 正常收到任务后，通常会先对评论添加反应，随后发布 Review。不要连续重复提交相同命令，否则可能生成重复任务并增加额度消耗。

### 5.3 自动审查

管理员可以在 Codex Code Review 设置中开启自动审查(已开)。开启后，新建 PR 会自动触发审查；已经存在的 PR 通常仍应通过 `@codex review` 手动触发。

Codex 审查会消耗 Codex 使用额度。可在 [Codex Usage](https://chatgpt.com/codex/settings/usage)查看使用情况；如不希望套餐额度耗尽后继续消耗额外 credits，应关闭额度超限使用或自动充值。

## 6. 让 Codex 修改代码或解决冲突

### 6.1 修复审查问题

在 Codex 的具体审查意见下回复，明确指出需要修复的内容：

```text
@codex fix this issue, keep the existing public API unchanged, run the available build checks, and push the fix to this pull request
```

如果有多条问题，建议明确优先级或逐条处理，不要只写“全部修复”而不说明行为边界。

### 6.2 解决与 `master` 的合并冲突

当 PR 显示 **Merge conflicts** 时，可以在 PR 中评论：

```text
@codex resolve the merge conflicts with master. Preserve the intended behavior of this branch, run the available build checks, and push the conflict-resolution commit to this pull request.
```

Codex 需要对仓库具有写入权限，才能把修复提交推送到 PR 分支。完成后必须再次检查 **Files changed**、构建结果和关键控制逻辑。

如果云端 Codex 未配置，也可以让本地 Codex 执行同样的工作，或手动操作：

```powershell
git fetch origin
git switch <开发分支名>
git merge origin/master
# 解决冲突并完成验证
git add <已解决的文件>
git commit
git push origin <开发分支名>
```共享开发分支优先使用 `merge`。除非团队已经协调，否则不要通过 `rebase` 后强制推送来重写其他开发者的提交历史。

## 7. 合并 PR

合并前确认：

- PR 不再是 Draft；
- 没有未解决的 Merge conflicts；
- 必要的 CI Checks 已通过；
- 人工审查和 Codex 审查中的阻塞问题已处理；
- 已完成本地编译，并清楚标注实机测试状态；
- PR 的 base 确实是 `master`。

确认无误后，可以在 GitHub 点击 **Merge pull request**，或使用：

```powershell
gh pr merge <PR编号> --merge
```

也可以明确要求本地 Codex 在检查上述条件后执行 `gh pr merge`。最终合并是远程写操作，必须给出明确的 PR 编号和授权；AI 审查或解决冲突不会自动等同于最终合并 PR。

## 8. 本机 `master` 自动更新 CI

工作流文件：`.github/workflows/auto-pull.yml`。

当 GitHub 上配置的目标分支（通常为 `master`）出现新 push 时，带有 `auto-pull` 标签的 Windows 自托管 Runner 会：

1. 检查目标目录和 `origin` 是否属于本仓库；
2. 获取远程目标分支；
3. 确认本地分支能够安全地 fast-forward；
4. 更新本地 `master` 引用。

它不会：

- 切换当前工作分支；
- 修改开发者当前工作区文件；
- 修改暂存区或未提交内容；
- 自动把最新 `master` 合入开发分支；
- 在本地和远程分叉时覆盖本地提交。

如果任意 worktree 正在检出 `master`、本地与远程已经分叉，或发现并发 Git 操作，工作流会安全失败并要求人工处理。

因此，开发分支仍需定期同步 `master`：

```powershell
git fetch origin
git switch <开发分支名>
git merge origin/master
```

管理员关于自托管 Runner、`AUTO_PULL_BRANCH` 和 `AUTO_PULL_PATH` 的配置方法见 `.github/CI_SETUP.md`。

## 9. 常见问题

### PR 没有自动添加标签或审查人

1. 确认 PR 不是 Draft；
2. 打开 PR 的 **Checks** 或仓库 **Actions** 查看 `Assign layer reviewers`；
3. 确认仓库 Actions Variables 已配置对应审查人；
4. 修改 PR 后重新 push，或关闭并重新打开 PR。

### `@codex review` 没有反应

1. 确认 Codex Code Review 已启用；
2. 确认 GitHub App 已授权本仓库；
3. 使用完全一致的 `@codex review` 新评论；
4. 检查 Codex 使用额度；
5. 授权是在评论之后补充的，应重新发送一条新评论。

### 自动更新 `master` 失败

1. 打开 Actions 中的 `Auto update local master ref` 日志；
2. 确认自托管 Runner 在线并具有 `Windows` 和 `auto-pull` 标签；
3. 确认 `AUTO_PULL_PATH` 和 `AUTO_PULL_BRANCH` 正确；
4. 确认没有 worktree 正在检出 `master`；
5. 如果本地 `master` 已分叉，人工确认并处理本地提交，不要强制覆盖。

### 如何停止审查

- 尚未启动且评论没有收到反应：删除 `@codex review` 评论即可；
- 已创建 Codex 任务：在对应任务页面使用 **Stop/Cancel**（如果页面提供）；
- 关闭自动审查只影响后续 PR，不保证终止已经运行的任务；
- GitHub 评论区没有可靠的 `@codex stop` 命令。

## 10. 安全原则

- 不要在 PR、日志、提交或 Codex 评论中粘贴密钥、Token 和设备凭据；
- 不要让来自不可信 Fork 的工作流在存有敏感信息的自托管 Runner 上执行；
- 不要让 AI 在未经检查的情况下修改电机限幅、保护逻辑、中断、CAN 或其他硬件关键代码；
- 不要使用 `git push --force` 覆盖共享分支；
- 不要因为 AI Review 通过就跳过编译、人工审查或必要的实机测试。

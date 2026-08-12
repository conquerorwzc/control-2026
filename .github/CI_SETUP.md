# CI 配置说明

本仓库包含两个 GitHub Actions 工作流：

- `assign-layer-reviewers.yml`：PR 修改 `Bsp/`、`Modules/`、`UserApp/` 时，自动添加层级标签并请求对应人员或团队审查。
- `auto-pull.yml`：指定分支发生 push 后，由 Windows 自托管 Runner 在后台获取远程提交，并仅将对应本地分支引用安全地快进。

## 1. 配置审查人

在 GitHub 仓库的 **Settings → Secrets and variables → Actions → Variables** 中添加以下仓库变量。多人或多个团队使用英文逗号分隔；未使用的变量可以留空。

| 变量 | 值的格式 | 示例 |
| --- | --- | --- |
| `BSP_REVIEWERS` | GitHub 用户名 | `alice,bob` |
| `BSP_TEAM_REVIEWERS` | 组织内 team slug | `bsp-maintainers` |
| `MODULES_REVIEWERS` | GitHub 用户名 | `carol` |
| `MODULES_TEAM_REVIEWERS` | 组织内 team slug | `module-maintainers` |
| `USERAPP_REVIEWERS` | GitHub 用户名 | `dave` |
| `USERAPP_TEAM_REVIEWERS` | 组织内 team slug | `app-maintainers` |
| `OTHER_REVIEWERS` | 其他所有路径的 GitHub 用户名 | `alice,bob` |
| `OTHER_TEAM_REVIEWERS` | 其他所有路径的 team slug | `repository-maintainers` |

团队审查只适用于 GitHub Organization 拥有的仓库。个人账号仓库请使用个人审查人变量。PR 作者本人会被自动排除。

`OTHER_REVIEWERS` 覆盖所有不在 `Bsp/**`、`Modules/**`、`UserApp/**` 下的改动，例如根目录文件、`CMakeLists.txt`、`Hardware/**` 和 `Utils/**`。如果一个 PR 同时修改层级目录和其他路径，会同时请求两类审查人。

如果审查必须通过才能合并，还需要在 **Settings → Branches/Rulesets** 中为目标分支启用 “Require a pull request before merging” 和 “Require approvals”。

## 2. 配置后台更新 master

1. 在目标 Windows 机器上安装 GitHub Actions self-hosted Runner。
2. 为 Runner 添加 `auto-pull` 标签，并将其作为该仓库的专用 Runner。
3. 确保 Runner 服务账号能够访问目标仓库目录及 GitHub 远程仓库。
4. 添加仓库变量：
   - `AUTO_PULL_BRANCH`：需要自动更新的分支，例如 `master`。
   - `AUTO_PULL_PATH`：本机仓库绝对路径，例如 `D:\github`。

工作流不会 checkout、merge 或修改开发者当前所在的分支。它只执行 fetch，并在满足 fast-forward 条件时原子地推进本地 `master` 引用。因此开发者可以留在其他分支工作，当前文件、暂存区和未提交修改不会被改变。

以下情况任务会安全停止：

- `master` 正在任意 Git worktree 中被签出；
- 本地 `master` 与 `origin/master` 已经分叉；
- 远程地址、目标目录或分支配置不匹配；
- 有其他 Git 操作与引用更新并发发生。

开发分支不会自动包含新的 `master` 提交。需要时仍应手动执行 `git rebase master` 或 `git merge master`。

> 安全提示：自托管 Runner 应仅用于受信任的仓库和分支。不要允许不受信任的工作流在存有敏感资料的个人机器上执行。

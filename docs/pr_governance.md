# 仓库协作审查结论与 PR 治理方案

## 当前主要问题（基于仓库现状）

1. 缺少统一 PR 自动化入口，历史上检查逻辑分散在多个 workflow 中。
2. PR 标题、构建测试结果、合并状态分散显示，失败后缺少固定汇总反馈。
3. 需要减少人工点选 approve / merge 的重复操作，同时保留基础治理能力。

## 当前已落地的最小治理方案

1. 统一为单一 `PR CI` workflow 处理 PR
2. 在同一 workflow 内保留语义化 PR 标题校验
3. 复用现有 ROS 构建与测试：工作区校验、`rosdep install`、`colcon build`、`colcon test`
4. 在 PR 中维护一条固定机器人汇总评论
5. 检查全部通过后由 bot 自动 approve，并启用 squash auto-merge
6. 保留 `.github/CODEOWNERS` 作为 ownership 提示，不再建议把它作为全自动合并的强制门禁
7. `CodeQL` 从 PR 触发中移除，只保留默认分支 push 与定时安全扫描

## 统一 PR workflow 说明

统一后的 `.github/workflows/pr-ci.yml` 分为两类触发：

- `pull_request`：执行不带写权限的代码检查 job
- `pull_request_target`：不执行 PR 代码，只等待检查完成后更新评论、自动审批、启用 squash auto-merge

这样做的目的：

- 同仓库 PR 与 fork PR 都能跑相同检查
- fork PR 的特权操作不直接运行 fork 代码，降低安全风险
- PR 页面只保留一个主要 workflow 作为合并门禁入口

## PR 自动化行为

### 检查阶段

统一 workflow 会执行以下稳定检查：

- `Semantic PR title`
- `ROS build and test`
- `PR gate`（聚合门禁 job，用于 branch protection）

其中标题格式要求为：

- `<type>: <description>`
- 允许类型：`feat`、`fix`、`docs`、`refactor`、`test`、`chore`、`ci`

### 汇总评论

workflow 会在 PR 中维护一条固定机器人评论，使用隐藏标记定位并更新，而不是每次新增一条新评论。

评论内容包含：

- 标题检查结果
- ROS build/test 结果
- `PR gate` 汇总结果
- 失败 job 列表
- 自动审批 / 自动合并执行结果
- 关联 workflow run 链接

### 自动审批与自动合并

当以下条件满足时，workflow 会尝试：

- 自动提交 bot approval
- 启用 squash auto-merge

前提条件：

- PR 不是 Draft
- 所有必需检查均通过
- 仓库允许对应 GitHub Actions 权限和 auto-merge 能力

补充说明：

- 当前实现把 fork PR 也纳入自动审批 / auto-merge 尝试范围
- 这依赖目标仓库的 Actions 权限策略、branch protection 配置，以及 GitHub 对 fork PR 的权限限制
- 因此“workflow 已尝试执行”与“仓库最终允许生效”是两件事，需要联调验证

## GitHub 仓库设置建议

在 `Settings -> General` 中确认：

- 开启 `Allow auto-merge`

在 `Settings -> Branches -> Branch protection rules` 中建议：

- Branch name pattern：填写默认分支（如 `master` 或 `main`）
- 开启 `Require a pull request before merging`
- 开启 `Require status checks to pass before merging`
- 先把 workflow 变更推到远端，并让 PR 至少运行一次；否则 GitHub 的 required checks 列表里不会出现新 check
- 必需检查只勾选：
  - `PR gate`
- 建议开启 `Require branches to be up to date before merging`

如果目标是“检查通过后全自动合并”，还需确认：

- 不存在阻塞 bot 的强制人工审批要求，或 bot approval 可以计入审批数
- 不存在阻塞 bot 的强制 Code Owner 审批要求
- 仓库针对 fork PR 的 Actions 权限策略允许评论、approve、auto-merge 所需权限

## CODEOWNERS 建议

保留 `.github/CODEOWNERS` 文件，继续用于：

- reviewer 请求提示
- ownership 声明
- 代码责任边界表达

但不建议在“希望 bot 自动 approve + auto-merge”的前提下，将 Code Owner review 作为强制合并门禁。

## 联调验证清单

1. 同仓库 PR，标题不规范
   - 统一 workflow 触发
   - `Semantic PR title` 失败
   - 固定评论更新为失败
   - 不执行自动合并

2. 同仓库 PR，build/test 失败
   - `ROS build and test` 失败
   - 固定评论更新并列出失败 job
   - 不执行自动合并

3. 同仓库 PR，修复后重新 push
   - 同一条评论更新为成功
   - bot 自动 approve
   - bot 启用 squash auto-merge

4. fork PR
   - 检查正常执行
   - 固定评论正常更新
   - workflow 尝试自动 approve / auto-merge
   - 最终是否生效取决于仓库权限与 GitHub 策略

5. 仓库设置联调
   - `Allow auto-merge` 已开启
   - workflow 已在 GitHub 上至少成功运行一次，因此 `PR gate` 已出现在 required checks 列表中
   - Branch protection required checks 已切换到 `PR gate`
   - 不存在阻塞 bot 的强制人工审批 / 强制 Code Owner 审批

6. 本地验证
   - 如环境允许，执行 `colcon build`
   - 如环境允许，执行 `colcon test`
   - 检查 workflow YAML 语法与条件逻辑

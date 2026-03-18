# 仓库协作审查结论与 PR 治理方案

## 当前主要问题（基于仓库现状）

1. 缺少 CI：没有自动构建、自动测试与自动安全扫描。
2. 缺少评审门禁：没有 CODEOWNERS 与统一 PR 模板。
3. 缺少提交规范：PR 标题和说明不统一，难以追踪变更意图。
4. 忽略规则不完整：容易误提交 IDE 文件与 Python 缓存。

## 已落地的最小治理方案

1. 新增 `PR CI` 工作流（`colcon build` + `colcon test`）
2. 新增 `PR Title Check` 工作流（语义化 PR 标题）
3. 新增 `CodeQL` 工作流（C++/Python 安全扫描）
4. 新增 `.github/CODEOWNERS`（核心路径默认请求 owner 评审）
5. 新增 `.github/pull_request_template.md`（统一 PR 提交流程）
6. 补充 `.gitignore`（IDE 与 Python 缓存）

## 合并前建议（GitHub 仓库设置）

在 `Settings -> Branches -> Branch protection rules` 中开启：

- Require a pull request before merging
- Require approvals（至少 1 人）
- Require status checks to pass before merging：
  - PR CI
  - PR Title Check
  - CodeQL

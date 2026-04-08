# 仓库访问失败说明 / Repository Access Failure Guide

## 问题描述

在尝试访问 `ChengYaofeng/A2G` 仓库时，服务器返回了 **HTTP 403 Forbidden** 错误，导致无法读取仓库内容。本文档解释了该错误的根本原因，以及可采取的解决步骤。

---

## 访问失败的原因

### 1. 仓库为私有（最常见原因）

GitHub 仓库的可见性分为三种：

| 可见性 | 说明 |
|--------|------|
| `Public`（公开） | 任何人均可读取，无需登录 |
| `Private`（私有） | 仅仓库所有者及被授权协作者可访问 |
| `Internal`（组织内部）| 仅同一 GitHub 组织成员可访问（Enterprise 功能） |

`ChengYaofeng/A2G` 返回 **403** 而非 **404**，说明：
- 该仓库**存在**，但当前会话没有读取权限
- 仓库很可能被设置为 **Private**

### 2. 访问令牌（Token）权限不足

GitHub API 通过 Personal Access Token（PAT）或 OAuth Token 鉴权。若令牌：
- 未包含 `repo` 权限范围（scope）
- 仅具有 `public_repo` 权限（无法访问私有仓库）
- 已过期或被撤销

则对私有仓库的任何请求都会返回 403。

### 3. 组织访问策略限制

若仓库归属于某个 GitHub 组织，组织可能启用了以下策略：
- **Third-party application restrictions**：限制第三方 OAuth App 访问组织资源
- **IP 允许列表**：仅特定 IP 段可访问组织资源
- **SAML SSO 强制认证**：令牌需完成 SSO 授权才能访问组织仓库

---

## 技术细节：HTTP 403 vs 404

| 状态码 | 含义 | 说明 |
|--------|------|------|
| `404 Not Found` | 仓库不存在，或私有仓库对未授权用户隐藏 | GitHub 有时用 404 代替 403，以避免暴露私有仓库的存在 |
| `403 Forbidden` | 仓库存在，但当前凭据无权访问 | 明确表示权限不足 |

---

## 可操作的下一步骤

### 方法 A：向仓库所有者申请访问权限

1. 联系 `ChengYaofeng`（仓库所有者），请求将你的 GitHub 账号添加为 **Collaborator**
2. 所有者路径：**仓库 Settings → Collaborators and teams → Add people**
3. 接受邀请后，使用具有 `repo` scope 的 PAT 重新尝试访问

### 方法 B：检查并更新你的访问令牌

1. 前往 **GitHub → Settings → Developer settings → Personal access tokens**
2. 确认令牌包含以下权限：
   - `repo`（完整仓库读写权限，含私有仓库）
   - 如果是组织仓库，确认令牌已通过 **SSO Authorization**
3. 生成新令牌后重新配置访问凭据

### 方法 C：使用本仓库作为替代参考

本仓库（`LianzhaoZhang/mit_ws`）是一个面向四足机器人的**公开** MPC 控制器实现，包含：
- 基于 MIT Cheetah 单刚体动力学方程的 MPC 控制器
- Unitree Go1 机器人接口与仿真启动文件
- Gazebo 仿真运行完整流程

如果你的目标是研究四足机器人运动控制（如步态规划、MPC/LQR 控制器、仿真环境搭建），可以直接基于本仓库展开，无需访问私有仓库。

### 方法 D：搜索同类公开仓库

如果 `ChengYaofeng/A2G` 是某篇论文或课题的相关代码，可尝试以下途径：
- 在 [Papers With Code](https://paperswithcode.com) 搜索论文标题查找公开实现
- 在 GitHub 搜索相关关键词（如 `quadruped A2G locomotion`）
- 联系论文作者请求代码访问权

---

## 常见问题

**Q：为什么返回 403 而不是 404？**  
A：GitHub 对于私有仓库通常返回 404 以隐藏其存在，但当会话携带了部分凭据（如过期或权限不足的令牌）时，服务器能识别出鉴权尝试，因此返回 403 明确告知权限不足。

**Q：如果仓库不存在会怎样？**  
A：GitHub 会返回 `404 Not Found`。收到 403 意味着仓库确实存在，只是当前账号没有访问权限。

**Q：组织仓库需要额外操作吗？**  
A：是的。即使你有 `repo` scope 的令牌，若该组织启用了 SAML SSO，你还需要在令牌页面点击 **"Authorize"** 完成 SSO 授权，否则仍会收到 403。

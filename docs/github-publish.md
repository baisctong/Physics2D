# 发布到 GitHub：检查清单与执行记录（v1.0-release）

> 面向仓库所有者（不是项目使用者）。目标：把这个仓库安全、体面地放到 GitHub（公开作品集或私有备份），
> 并让 CI / Release / README 在 GitHub 上看起来是"完成品"。
>
> 仓库目标：**https://github.com/baisctong/Physics2D**（账号 `baisctong`，邮箱 `1519892588@qq.com`）。

## 0. 当前状态（2026-10-31 执行后）

| 项 | 状态 | 说明 |
| --- | --- | --- |
| tracked 文件数 / `.git` 体积 | ✅ 168 / ≈3.4 MiB | 无需 LFS、无需清理历史大文件 |
| `.gitignore` | ✅ | 忽略 `build/`、`*.log`、IDE 文件、`.DS_Store`/`Thumbs.db`/`*.ppm`；白名单 `docs/**/*.gif` |
| `.gitattributes` | ✅ | 统一 LF/CRLF、GIF/PNG/PPM 标记 binary |
| LICENSE | ✅ MIT | 版权人已设为 `baisctong` |
| README | ✅ 中英双语 + GIF 墙 + 真实数据 + CI/Release badge | 相对链接全部自检通过；badge 指向 `baisctong/Physics2D` |
| Mermaid 图 | ✅ 11/11 通过 Mermaid 11 语法校验 | GitHub 上不会出现报错框 |
| 本地绝对路径 | ✅ 已清理 | 文档里的本机路径替换为 `<repo-root>` / 相对路径 |
| **Git 身份 / 历史** | ✅ **已改写** | 全部 commit + 8 个 tag 的 author/committer/tagger 已改为 `baisctong <1519892588@qq.com>` |
| remote | ✅ 已添加 | `origin = https://github.com/baisctong/Physics2D.git`（推送前需仓库已存在 + 本机 GitHub 登录） |
| 推送 main + tags | ⏳ **待执行** | 需要你本机完成一次 GitHub 登录（见 §2），或提供 PAT |
| GitHub Release / About / topics | ⏳ 待执行（网页/API） | 见 §3 |
| CI 首跑 | ⏳ 推送到 GitHub 后触发 | 从未在远端跑过，首推要盯（见 §5） |

## 1. Git 身份（已完成，保留命令供参考）

目标身份：`baisctong <1519892588@qq.com>`（GitHub 账号 `baisctong`）。
已完成：设置本仓库 `user.name`/`user.email`，并用 `git filter-repo --mailmap` 把全部历史
（commit author/committer + annotated tag 的 tagger）从 `Physics2D Dev <dev@physics2d.local>`
改写为 `baisctong <1519892588@qq.com>`。

如需再次确认：

```powershell
git log --format="%an <%ae> | %cn <%ce> | %s" | Select-Object -First 3
git rev-list --count HEAD      # 44
git tag --list                 # 8 个 tag
git describe --tags            # v1.0-release
git status --short             # 空
```

如果以后要改显示名（例如用真实姓名替代 `baisctong`），可以再跑一次同样的 mailmap 流程
（`pip install git-filter-repo` → `git filter-repo --mailmap mailmap.txt --force`），
但**推送之后不要再改写历史**（会让别人 clone 的 hash 失效）。

> 注意：历史改写会改变所有 commit hash；当前还没有远端，所以是零成本的安全操作。
> 若你希望 commit 关联 GitHub 贡献图，请确认 `1519892588@qq.com` 已加入 GitHub 账号
> （Settings → Emails）；否则 GitHub 会把 commit 显示为未关联。

## 2. 创建远端仓库并推送（待执行，需要一次 GitHub 登录）

### 2.1 先在 GitHub 创建空仓库

网页：<https://github.com/new> → Repository name 填 **Physics2D** → **不要**勾选
README / .gitignore / LICENSE（避免与本地历史冲突）→ Create repository。

> 当前 API 查询 `GET /repos/baisctong/Physics2D` 返回 404，说明仓库尚未创建（或为私有）。
> 如果建私有仓库也可以推，但 badge 图片对匿名访问会 404；作品集建议 public。

### 2.2 推送（在本机仓库根目录执行）

```powershell
cd "<repo-root>"

git remote -v                       # 应显示 origin https://github.com/baisctong/Physics2D.git
git status --short                  # 空
git push -u origin main             # 首次会弹出 Git Credential Manager 的 GitHub 登录（浏览器/设备码）
git push origin --tags              # 8 个 tag，含 v1.0-release
git describe --tags                 # v1.0-release
```

若 `origin` 不存在：`git remote add origin https://github.com/baisctong/Physics2D.git`。
若用 SSH：`git remote set-url origin git@github.com:baisctong/Physics2D.git`（需先上传 SSH key）。

> 本机 Git Credential Manager 目前**没有**保存 GitHub 凭据（已探测），所以第一次 push 需要在弹窗里登录
> `baisctong` 账号；登录一次后凭据由 Windows 凭据管理器保存。若不想交互，可用 PAT：
> `git remote set-url origin https://<PAT>@github.com/baisctong/Physics2D.git`（推送后记得改回干净 URL）。

## 3. GitHub 仓库设置（网页，5 分钟）

- **About → Description**（英文，可直接复制）：
  `A 2D rigid-body physics engine written from scratch in C++20: broad phase → SAT/clipping manifold → Sequential Impulse → islands/sleeping. Zero third-party physics deps, headless deterministic regression, 8-demo ImGui testbed.`
- **About → Topics**（建议）：
  `cpp20` `physics-engine` `rigid-body-dynamics` `game-physics` `sequential-impulse` `sat`
  `broad-phase` `sweep-and-prune` `collision-detection` `opengl` `imgui` `cmake` `googletest`
- **Social preview**：上传 1280×640 图（可从 `docs/images/box_stack.gif` 取一帧，或截 testbed 画面）；
- **Release**：`Releases → Draft a new release → Choose tag v1.0-release` → 标题
  `v1.0-release — Week 1-8 complete` → 正文粘贴 [`release-notes-v1.0.md`](release-notes-v1.0.md)
  →（可选）把 4 张 GIF 拖进正文，GitHub 会转存到 user-images，不占仓库体积；
- 可选：pin 仓库、Settings → Branches 保护 `main`（要求 CI 通过）、关闭不需要的 Discussions/Wiki。

## 4. README badges（已添加，推送后自动生效）

两行已写入 [`../README.md`](../README.md) 与 [`../README.en.md`](../README.en.md) 标题下：

```markdown
[![CI](https://github.com/baisctong/Physics2D/actions/workflows/ci.yml/badge.svg)](https://github.com/baisctong/Physics2D/actions/workflows/ci.yml)
[![Release](https://img.shields.io/github/v/release/baisctong/Physics2D?label=release)](https://github.com/baisctong/Physics2D/releases)
```

CI badge 在第一次 workflow 运行完成后才会出状态；Release badge 在 tag 推上去后显示 `v1.0-release`。

## 5. 首次推送后：盯一次 CI（重要）

**这个仓库的 CI 从未在远端跑过**（W1 起所有验证都是本机脚本）。6 个 job 定义、GitHub UI 上共 9 条检查：
`build-test`（ubuntu/windows × debug/release = 4）、`sanitizers`、`headless`、`regression`、`format`、`layering`。
首推后预计需要 1 次调试迭代，优先看：

1. **format**：CI 的 clang-format **固定为 `23.1.3`**（与本机一致；ubuntu apt 包版本更老，可能对少数
   构造给出不同结果）。若 pip 装包失败，可改 `pipx run clang-format==23.1.3` 或临时设为非阻塞（不推荐）。
2. **sanitizers（Linux）**：31 个 death test 会**真正执行**（本机 sandbox 里被 SKIP，只在 debug 工作区外
   验证过）。若个别在 ASan/UBSan 下行为不同，看 `ctest --output-on-failure`；必要时先确认不是真实泄漏，
   再考虑 `ASAN_OPTIONS=detect_leaks=0`。
3. **build-test / windows**：首次 FetchContent 下载 + MSVC 编译 GLFW/ImGui 较慢（正常 5–10 分钟）；
   若 `cl.exe` 找不到，检查 `ilammy/msvc-dev-cmd` 是否在 configure 之前（当前是）。
4. 若 CI 红而本地绿：按 [`testing.md`](testing.md) §2 复跑同一 preset，再开 PR / `workflow_dispatch` 调试；
   修 CI 不影响已发布 tag 的内容。

## 6. 发布前最后自检（本次已全部通过）

```powershell
git status --short                 # 空
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/format.ps1 -Check   # 117 文件 OK
.\build\debug\tests\physics2d_regression.exe --check tests\regression\golden\default_600.txt  # 588d30e70f9b4ddd
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests   # 387/387
```

已完成的 GitHub 就绪改动：
- `.gitattributes`（LF/CRLF 归一、二进制标记）+ `.gitignore` 补充跨平台杂物；
- CI：只读权限、`workflow_dispatch`、job 超时、format job 固定 clang-format 23.1.3；
- 清理文档本地绝对路径、删除冗余 `docs/images/.gitkeep`；
- 修复 2 处 Mermaid 语法（GitHub 会显示错误的地方），11 个图全部通过语法校验；
- `LICENSE` 版权人、README 中英 badge、Git 历史与 tag 身份改写、remote 添加。

## 7. 不需要做的事（避免过度优化）

- **不需要 Git LFS**：仓库 ≈3.4 MB，最大文件 592 KB；GIF 已压缩到单张 ≤600 KB / 总计 1.69 MB。
- **不需要清理 `.git` 历史**：没有提交过 build 产物/大文件/密钥（44 个 commit 全部是源码+文档）。
- **不需要改 CMake/代码**：CI 用到的 preset/option 全部存在（`debug/release/sanitize/headless`、
  4 个 test preset、`PHYSICS2D_HEADLESS_ONLY` 等）。
- **不要在推送后为了"更好看"改写历史或改物理/调参**：v1.0 的标签意义在于可复现的验证基线。

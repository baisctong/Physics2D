# 发布到 GitHub：检查清单（v1.0-release）

> 面向仓库所有者（不是项目使用者）。目标：把这个仓库安全、体面地放到 GitHub（公开作品集或私有备份），
> 并让 CI / Release / README 在 GitHub 上看起来是"完成品"。
>
> 当前状态（2026-10-31 体检）：**可以直接推送**——167 个 tracked 文件、`.git` ≈ 3.2 MiB、
> 最大单文件 592 KB（`docs/images/sap_stress.gif`），**不需要 Git LFS**；工作区干净、tag `v1.0-release` 已就绪。

## 0. 体检结论

| 项 | 状态 | 说明 |
| --- | --- | --- |
| tracked 文件数 / `.git` 体积 | ✅ 167 / ≈3.2 MiB | 无需 LFS、无需清理历史大文件 |
| `.gitignore` | ✅ | 忽略 `build/`、`*.log`、IDE 文件；白名单 `docs/**/*.gif` |
| `.gitattributes` | ✅ 本次新增 | 统一 LF/CRLF、GIF/PNG 标记为 binary，杜绝跨平台换行 churn |
| LICENSE | ✅ MIT | 建议把 `Copyright (c) 2026 Physics2D contributors` 改成你的名字 |
| README | ✅ 中英双语 + GIF 墙 + 真实数据 | 相对链接全部自检通过（16 个 md 文件） |
| Mermaid 图 | ✅ 11/11 通过 Mermaid 11 语法校验 | 修复了 2 处 GitHub 会显示错误的写法 |
| 本地绝对路径 | ✅ 本次清理 | 文档里的 `E:\Big Fat Fish's Workstation\...` 已替换为 `<repo-root>` |
| CI workflow | ✅ 7 个 job；本次加固 | 但 **CI 从未在远端跑过**（见 §5，首推要盯） |
| Git 身份 | ⚠️ 需要你决定 | 42 个 commit + 8 个 tag 的作者都是本地身份 `Physics2D Dev <dev@physics2d.local>`，不会关联你的 GitHub 账号（见 §1） |
| GitHub Release / About / badges | ⚠️ 网页操作 | 见 §3、§4 |

## 1. Git 身份：唯一建议"改写历史"的一步

当前所有 commit 的 author/committer 都是 `Physics2D Dev <dev@physics2d.local>`。GitHub 按 **email**
把 commit 关联到账号；这个邮箱不属于任何账号，所以贡献图/头像不会显示你。

**⚠️ 现在是最安全的改写时机：远端还没建，改写只影响本地，不会破坏任何人的 clone。**

### 方案 A（推荐）：改写全部历史 + 以后用 GitHub noreply 邮箱

```powershell
# 1) 安装 git-filter-repo（Python 包；本机 python 可用）
pip install git-filter-repo

# 2) 写 mailmap.txt（UTF-8，按你的 GitHub 用户名/邮箱替换）
#    格式：新名字 <新邮箱> 旧名字 <旧邮箱>
@"
Your Name <12345678+yourname@users.noreply.github.com> Physics2D Dev <dev@physics2d.local>
"@ | Set-Content -Encoding UTF8 mailmap.txt

# 3) 设置今后使用的身份
git config user.name "Your Name"
git config user.email "12345678+yourname@users.noreply.github.com"

# 4) 重写 author/committer（包括 8 个 annotated tag 指向的 commit）
git filter-repo --mailmap mailmap.txt --force

# 5) 自检：作者已替换、42 个 commit、8 个 tag 仍在、工作区干净
git log --format="%an <%ae> %s" | Select-Object -First 3
git rev-list --count HEAD
git tag --list
git describe --tags
git status --short
```

> `git filter-repo` 会移除 `origin`（防误推旧历史）；因为还没加 remote，正好。
> 改写后 commit hash 全变；`v1.0-release` 由 filter-repo 自动重映射，指向新历史。

### 方案 B：不改历史（保留原样，最省事）

只设置今后身份，接受"旧 42 个 commit 不关联账号"：

```powershell
git config user.name "Your Name"
git config user.email "12345678+yourname@users.noreply.github.com"
```

适合把仓库当"私有备份"或不在意贡献图的情况；公开作品集建议方案 A。

### 同时建议

- 把 `LICENSE` 的 `Copyright (c) 2026 Physics2D contributors` 改成你的名字/ID；
- 在 GitHub Settings → Emails 里确认 `user.name`/`user.email` 与账号一致；
- 若你不想公开真实邮箱，用 GitHub 提供的 `ID+username@users.noreply.github.com`（Settings → Emails 可查）。

## 2. 创建远端仓库并推送

```powershell
# 在 GitHub 网页新建一个【空】仓库：不要勾选 README / .gitignore / LICENSE
# 仓库名建议 physics2d（小写，URL 更干净；本地目录名大小写无关）
git remote add origin https://github.com/<yourname>/Physics2D.git
git push -u origin main
git push origin --tags          # 8 个 tag（含 v1.0-release）必须单独推
git describe --tags             # 期望 v1.0-release
```

若用 SSH：`git remote add origin git@github.com:<yourname>/Physics2D.git`。

## 3. GitHub 仓库设置（网页，5 分钟）

- **About → Description**（英文，建议直接复制）：
  `A 2D rigid-body physics engine written from scratch in C++20: broad phase → SAT/clipping manifold → Sequential Impulse → islands/sleeping. Zero third-party physics deps, headless deterministic regression, 8-demo ImGui testbed.`
- **About → Topics**（建议）：
  `cpp20` `physics-engine` `rigid-body-dynamics` `game-physics` `sequential-impulse` `sat`
  `broad-phase` `sweep-and-prune` `collision-detection` `opengl` `imgui` `cmake` `googletest`
- **Social preview**：上传一张 1280×640 截图（可从 `docs/images/box_stack.gif` 取一帧或自己截 testbed）；
- **Release**：`Releases → Draft a new release → Choose tag v1.0-release` → 标题
  `v1.0-release — Week 1-8 complete` → 正文直接粘贴 [`release-notes-v1.0.md`](release-notes-v1.0.md) 的内容
  → 可把 4 张 GIF 拖进正文（GitHub 会转存到 user-images，不占仓库体积）；
- 可选：pin 这个仓库、Settings → Branches → 保护 `main`（要求 CI 通过）、关闭不需要的 Discussions/Wiki。

## 4. README badges（推送后加两行）

CI 跑过一次后，在 [`../README.md`](../README.md) 与 [`../README.en.md`](../README.en.md) 标题下加：

```markdown
[![CI](https://github.com/<yourname>/Physics2D/actions/workflows/ci.yml/badge.svg)](https://github.com/<yourname>/Physics2D/actions/workflows/ci.yml)
[![Release](https://img.shields.io/github/v/release/<yourname>/Physics2D?label=release)](https://github.com/<yourname>/Physics2D/releases)
```

> badge 在 CI 从未运行时会显示 "no status"/灰色，所以**先推代码、确认 CI 绿、再加 badge**。

## 5. 首次推送后：盯一次 CI（重要）

**这个仓库的 CI 在本地从未真正跑过**（GitHub Actions 需要远端仓库；W1 起所有验证都是本机脚本）。
7 个 job：`build-test`（ubuntu/windows × debug/release 共 4 个矩阵）、`sanitizers`、`headless`、
`regression`、`format`、`layering`。首推后预计需要 1 次调试迭代，优先看：

1. **format**：已把 CI 的 clang-format **固定为 `23.1.3`**（与本机一致；发行版 apt 包版本更老，可能对
   少数构造给出不同结果）。若 pip 装包失败，改成 `pipx run clang-format==23.1.3 --dry-run ...` 或
   临时把该 job 设为非阻塞（不推荐长期）。
2. **sanitizers**：Linux 上 31 个 death test 会**真正执行**（本机 sandbox 里被 SKIP，只在 debug
   工作区外验证过）。若个别 death test 在 ASan/UBSan 下行为不同，看
   `ctest --output-on-failure` 的具体失败；必要时在 CI 里给 death test 设 `ASAN_OPTIONS=detect_leaks=0`
   （先确认不是真实泄漏）。
3. **build-test / windows**：首次 FetchContent 下载 + MSVC 编译 GLFW/ImGui 较慢（正常 5–10 分钟）。
   若 `cl.exe` 找不到，检查 job 顺序里 `ilammy/msvc-dev-cmd` 是否在 configure 之前（当前是）。
4. 若 CI 红而本地绿：先按 [`testing.md`](testing.md) §2 在本机复跑同一 preset，再开一个 PR 触发
   `workflow_dispatch` 调试；修 CI 可以放在 tag 之后（不影响 `v1.0-release` 内容）。

## 6. 发布前最后自检（本次已全部通过）

```powershell
git status --short                 # 空
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/format.ps1 -Check   # 117 文件 OK
.\build\debug\tests\physics2d_regression.exe --check tests\regression\golden\default_600.txt  # 588d30e70f9b4ddd
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests   # 387/387
```

本次为 GitHub 就绪做的仓库级修改：
- 新增 `.gitattributes`（LF/CRLF 归一、二进制标记）；
- CI：`permissions: contents: read`、`workflow_dispatch`、各 job `timeout-minutes`、format job 固定
  clang-format 23.1.3；
- 清理文档中的本地绝对路径（HANDOFF/testing/profiling/resume-interview/dev-setup）；
- 删除冗余的 `docs/images/.gitkeep`；
- 新增本文档；修复 2 处 Mermaid 语法（GitHub 上会显示错误的地方）。

## 7. 不需要做的事（避免过度优化）

- **不需要 Git LFS**：仓库 3.2 MB，最大文件 592 KB；GIF 已压缩到单张 ≤600 KB / 总计 1.69 MB。
- **不需要清理 `.git` 历史**：没有提交过 build 产物/大文件/密钥（42 个 commit 全部是源码+文档）。
- **不需要改 CMake/代码**：CI 用到的 preset/option 全部存在（`debug/release/sanitize/headless`、
  4 个 test preset、`PHYSICS2D_HEADLESS_ONLY` 等）。
- **不要为了"更好看"在发布后改物理/调参数**：v1.0 的标签意义在于可复现的验证基线。

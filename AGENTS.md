# Working on this repo (iOS on Redmi 10C, `rain`)

- Read `PORTING-rain.md` before starting: hardware facts, the boot log of every test
  image, the current blocker and the next step. `NOTES.md` and `docs/UPSTREAM-topaz.md`
  are the upstream projects' logs (other phones).
- Repo: github.com/taynotfound/ios-rain, branch `rain` (remote `fork`; `origin` is upstream MakrSas/ios-topaz). Kernel builds: `gh workflow run xnu.yml -R taynotfound/ios-rain --ref rain`; GPU user space: `userland.yml`.
- The user switches agents frequently. Update `PORTING-rain.md`
  after each milestone and before stopping. Record commits, CI run IDs, exact
  build/boot results and unfinished work. Distinguish observations from guesses.
- Commit completed changes. Use `gh` for GitHub Actions kernel builds and logs.
  Local single-file cross-compiles are useful before spending a CI run.
- Device bring-up uses temporary `fastboot boot` images. Preserve the Android
  installation and partition backups. The user approved (2026-10-15) using
  internal UFS for iOS and said Android may be deleted: the iOS root goes
  into the existing `userdata` partition (no GPT changes). Keep bootloaders,
  recovery, modem, persist and the backups intact, and confirm each
  destructive step with the user before running it.
- Keep device dumps, serial numbers and downloaded Apple artifacts out of git.

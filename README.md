# Setup recommendations

### 1. Configure remotes
- `origin` - Your personal GitHub fork. This will be used to create pull requests to the upstream (team) repository.

- `upstream` - The team's main repository. This will be used to _pull_ updates into your local `main` branch.

### 2. Disallow commits to the main branch
This makes sure that the main branch never diverges from the upstreeam branch. Your commit will be rejected, as a reminder to create a new branch.
- Copy the files(s) from the githooks folder to .git\hooks \
`copy githooks/* .git/hooks`

### 3. Configure the `main` branch to pull from upstream, and prohibit pushing.
After a Pull Request is merged into the upstream repository, you will want to be able to easily pull the updated main branch to your local machine.
```
git config branch.main.remote upstream
git config branch.main.pushRemote no_push
git config remote.pushDefault origin
```
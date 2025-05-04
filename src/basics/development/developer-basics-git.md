```{seo}
:description: Comprehensive guide to using Git and GitHub within the Duckietown modular software ecosystem, covering branches, commits, pull requests, and best practices.
:keywords: Git, GitHub, version control, Duckietown, branches, commits, pull requests, software modularity, development workflow
```

(sec:developer_basics_git)=
# Git

Every time there is a large project, with many contributors and
the need for code versioning and history, developers rely on VCS
(Version Control Systems) tools.
Duckietown uses **Git** as VCS and [GitHub](https://github.com) 
as a service provider for it. The Duckietown organization page on GitHub is 
[github.com/duckietown](http://github.com/duckietown).


## Monolithicity versus Modularity

Whether a software project should be monolithic or modular is
one of the most debated decisions that a group of developers
faces at the beginning of a software project. Books have been 
written about it. Duckietown started as a monolithic project, 
and some of us still remember the infamous 
[Software](http://github.com/duckietown/Software) repository,
and only later transitioned to a full modular approach.

Duckietown distinguishes between **modules** and **nodes**:

* A **module** is a high-level package (for example, _autonomous-driving_).

* A **node** is the smallest software entity, usually responsible for one task (for example, _traffic-sign-detection_). Nodes always reside inside modules.

Each module has its own repository under the Duckietown GitHub organization. Personal or course-specific modules can live in separate GitHub accounts while remaining fully compatible with the ecosystem.

```{tip}
Be brave, Duckietown has hundreds of repositories.
```
(dev-basics-git-terminology)=
## Git

The most frequently used Git concepts are summarised below.

### **Repository**

A repository (or _repo_) holds the complete file set for a project plus the entire revision history, i.e., the history of every change done on each file. 


### **Branch**

Branches constitute threads of changes within a repository. Though we
call them branches, do not try to force an analogy with tree 
branches, they are similar in spirit but quite different in how 
they work.

A repository is not allowed to exist without a branch, and
every operation inside a repository only makes sense in the context
of a branch (the _active_ branch).
A repository can have many branches, but only one active at the time.

Every Git project has at least one main branch, usually called 
the `master` or `main` branch.

Branches are used in different scenarios, allowing different developers to work simultaneously on their own task without having their work affect or be affected by the others.

For example, after a project is released with version `1.0.0`, one team might be tasked to develop a new feature for the version `1.1.0` milestone, while another team is asked to fix a user-reported bug which be patched in a version `1.0.1`.

Branch operations, like switching between branches or checking which is the active branch, are performed through the command `git branch`.

### **Commit**

A commit is an atomic set of changes recorded in the repository history and is identified by a unique `SHA-1` hash. 

When you create/delete/edit one or more files in a repository, 
and you are confident enough about those changes, you can commit (or, "save" them to the online version system) them to the branch using the workflow:
* `git add <file(s)>`, e.g., `git add .` to indicate all changes
* `git commit -m "Replace me with some informative message regarding the commit."`.

```{note}
A commit is not a snapshot (or a copy) of the entire repository
at a given point in time. Each commit contains only the incremental
difference from the previous commit, called *delta* in Git.
``` 

A chain of commits in which all the ancestors are included makes a 
branch. Since every commit is linked to its parent, a branch is
simply a *pointer* to a commit (the full chain of commits can always
be reconstructed from the commit).
In other words, you can think of branches as human friendly labels
for commits. Every time you create a new commit, the pointer of the
current branch advances to the newly created commit.


### **Tag**

A tag is a human friendly name for a commit but unlike branches, tags
are read-only. Once created, they cannot be modified to point to
a different commit.

Tags are commonly used for labeling commits that constitute 
milestones in the project development timeline, for example a release.


### **Fork**

A fork is basically a copy of someone else's repository. 
Usually, you cannot create branches or change code in other 
people's repositories, that's why you create your own copy of it. 
This is called `forking` and allows you to experiment without affecting the original.


### **Remote**

A git *remote* is a copy of your repository hosted by a Git service
provider, e.g. [GitHub](https://github.com). Remotes allow you to
share (`push`) your commits and branches so that other developers can `fetch`
them. Remotes are the same as local repositories, but they are reachable 
over the internet.

You can use the commands `git fetch` and `git push` to bring your
local copy of the repository in sync with a remote, by downloading
commits or uploading new commits respectively.


### **Merging branches**

Merging integrates the history of one branch into another. Conflicts occur when the same lines of code differ between branches and must be resolved manually.

Imagine having branched out from `master` to `new-patch` to deploy a new bug fix. Now you want to incorporate these changes back in the `master` branch, which hosts your main code. The **merge** operation does exactly that. It takes the changes done in `new-patch` and applies them
to `master`.

Often Git will manage to apply these changes seamlessly. However,
if both `new-patch` and `master` have changes to the same line of code, Git
will not be able to determine automatically which of the two changes should be preserved. These are called _merge conflict_ and will have to be solved manually, by selecting which of the two version to keep. 
    
### **Pull Requests**

A pull request (or, **PR**) on GitHub proposes that changes from one branch (often in a fork) be merged into another branch and provides a discussion and review interface before the merge is executed.

A pull request can be seen as a three-step merge operation between two 
branches where the changes are first _proposed_, then _discussed and adapted_
(if requested), and finally _merged_.

PRs are a better practice than direct merging, as it provides an extra-layer of revision before potentially compromising the main code with undesirable changes. 

## Common operations

### **Fork a repository on GitHub**

To fork a repo you have to go to the repository's webpage and click 
on the fork button in the upper right corner.


### **Clone a repository**

Cloning a repository is the act of creating a local copy of a remote
repository. A repo is cloned only at the very beginning, when you
still don't have a local copy of it.

To clone a repository, either copy the HTTPS or SSH URL given on 
the repository's webpage, then:

    git clone [REPOSITORY-URL]
    
This will create a directory in the current working path with 
the same name of the repository and the entire history of commits 
will be downloaded onto your computer.


### **Create a new branch**

The command for creating a new branch is a little bit 
counter-intuitive, but you will get use to it:

    git checkout -b [NEW-BRANCH-NAME]
    
This creates a new branch pointing at the same commit your
currently active branch is pointing at. In other words, you will
end up with two branches pointing at the same commit. Note that
after you issue this command, the newly created branch becomes
your active branch.

### **Inspecting working tree status**

In Git, the term *working tree* indicates all the changes 
that are not committed yet. You can think of it as your workspace.
When you create a new commit, the hash for the current working tree 
is computed and assigned to the new commit together with the changes
since the last commit. The working tree clears as you commit changes.

```{note}
You cannot create commits from a clean working tree.
```

Use the command `git status` to inspect the status of your working
tree.


### **Create a new commit**

A commit is created through a two-step process. First, mark all the 
changes that will be part of the new commit:

    git add <file(s)>

```{tip}
The command `git status` will always show you which changes are 
marked to be used for a new commit and which changes are not.
```
Second, create the actual commit:

    git commit -m "Replace me with a really informative message."

to create a new commit. Replace `[COMMIT-MESSAGE]` with your
notes about what changes this commit includes.

```{warning}
Do not underestimate the value of informative commit messages. When looking back to a repo's history of commits, looking for a change of interest, having good commit messages will be a game changer.
```

### **Push changes**

Use the following command to *push* your local changes to the remote
repository so that the two repositories can get in sync.

    git push origin [BRANCH-NAME]


### **Fetch changes**

If you suspect that new changes might be available on the remote
repository that are not synced to your local repo, you can use the command 

    git fetch origin [BRANCH-NAME]

to download the new commits available on the remote (if any).
These new changes will be appended to the branch called 
`origin/[BRANCH-NAME]` in your local repository.

If you want to apply them to your current branch, use the command

    git merge origin/[BRANCH-NAME]
    
Use the command `git pull origin/[BRANCH-NAME]` to perform *fetch*
and then *merge*.


### **Delete a branch locally and remotely**

Unlike the vast majority of git commands, the delete one does not
work on the current branch. You can delete other branches with:

    git branch -d [BRANCH-NAME]

If you want to delete your current branch, you will need to checkout
another branch first. This prevents ending up with a repository
with no branches.

To propagate the deletion of a branch to the remote repository,
run the command:

    git push origin --delete [BRANCH-NAME]

### **Open a GitHub Issue**

If you are experiencing issues with any code or content of a repository, 
such as this manual, you can submit GitHub "issues" by navigating to the `Issues` tab of the desired repo and selecting _New issue_.

For example, when you find a typo or a mistake in this instruction manual, 
you can (and should!) visit the [Duckietown developer manual issues page](https://github.com/duckietown/book-devmanual-docs/issues) and report an issue, so it can be reviewed and addressed. 

Even better, you could fork this repo, fix the issue, and make a PR. We have a different manual with [tips and tricks on how to contribute to the Duckietown documentation system](https://docs.duckietown.com/daffy/devmanual-docs/intro.html).   

GitHub Issues are a crucial part of the lifecycle of a software product, as
they provide a feedback loop that goes directly from the end-user to the  developers. You do not have to be a developer or an expert in software
engineering to open an Issue, and issue cannot "break the build", so you can dare and open issues even when you are not super confident. 


## Hands-On resources

(dev-basics-git-resources)=
### Git

It is strongly recommended that all Git beginners follow this well done tutorial:

* [Learn Git Branching](https://learngitbranching.js.org/).

Further reading material can be found at the following links:

* [Git Handbook](https://guides.github.com/introduction/git-handbook/)
* [Basic Branching and Merging](https://git-scm.com/book/en/v2/Git-Branching-Basic-Branching-and-Merging)
* [GitHub hello world](https://guides.github.com/activities/hello-world/)

(dev-basics-git-github-setup)=
### GitHub essentials

To get the ball rolling, create a (free) account at https://github.com and configure your SSH keys for password-less access:

1. [Generate a new SSH key](https://help.github.com/en/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
2. [Add SSH key to your GitHub account](https://help.github.com/en/github/authenticating-to-github/adding-a-new-ssh-key-to-your-github-account).


## Ask the community

If you have any questions about how to use of Git in Duckietown,
[join the Duckietown Slack community](https://duckietown.com/join-slack) and ask away.


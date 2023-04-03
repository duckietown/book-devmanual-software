(sec:developer_basics_git)=
# Git

Every time there is a large project, with many contributors and
the need for code versioning and history, developers rely on VCS
(Version Control Systems) tools.
Duckietown uses Git as VCS and [GitHub.com](https://github.com) 
as a service provider for it.
The Duckietown organization page on GitHub is 
[github.com/duckietown](http://github.com/duckietown).


## Monolithicity VS Modularity

Whether a software project should be monolithic or modular is
one of the most debated decisions that a group of developers
faces at the beginning of a software project. Books have been 
written about it. Duckietown started as a monolithic project, 
and some of us still remember the infamous 
[Software](http://github.com/duckietown/Software) repository,
and only later transitioned to a full modular approach.

There are two levels of modularity in Duckietown. We distinguish
between **Modules** and **Nodes**. Modules form our first and highest 
level of modularity, with each module being a collection of nodes.
Nodes constitute the smallest software entities, and each node is
usually responsible for a very specific task. Nodes are not allowed
to exist outside modules.
We will revisit these concepts later in the book, but in a nutshell,
modules represent high level concepts, like _autonomous driving
capability_ for a vehicle, while nodes within a module tackle more
granular tasks, like _traffic signs detection_.

In Duckietown, code is separated so that each module has its own
repository. All official repositories are hosted under the same GitHub
organization [github.com/duckietown](http://github.com/duckietown).
Be brave, (as of April 2020) we have more than 220 repositories 
there. You can also have your own modules hosted as repositories
on your own GitHub account.


## Git

This section goes through the most common operations you can 
perform on a git project and a git project hosted on GitHub.com.

### Terminology

A non-exhaustive list of terms commonly used in git follow.

#### **Repository**

A repo (short for repository), or git project, encompasses the 
entire collection of files and folders associated with a project, 
along with each file’s revision history.


#### **Branch**

Branches constitute threads of changes within a repository. Though we
call them branches, do not try to force an analogy with tree 
branches, they are similar in spirit but quite different in how 
they work.

A repository is not allowed to exist without a branch, and
every operation inside a repository only makes sense in the context
of a branch (the _active_ branch).
Inside a repository, you can have as many branches as you want, 
but you always work on one branch at a time.
Every git project has at least one main branch, usually called 
the `master` branch.

Use the command `git branch` to see the list of branches present
in your repository and which branch you are currently working on.

Though, branches are used in different scenarios, they simply
allow groups of developpers to work on their own task without
having their work affect or be affected by other groups' work.
For example, after a project is released with version `1.0.0`,
one team is tasked to develop a new feature for the version `1.1.0`
milestone while another team is asked to fix a bug that a user
reported and whose patch will be released in the version `1.0.1`.

Branch operations are performed through the command `git branch`.


#### **Commit**

A commit is an atomic change in a repository. A commit is a set of
changes to one or more files within your repository. Each commit is 
uniquely identified within a repository by the hash (SHA-1) of the 
changes it contains ("plus" a header).

When you create/delete/edit one or more files in a git repository 
and you are confident enough about those changes, you can commit them
using the command `git commit`.

Note: A commit is not a snapshot (or a copy) of the entire repository
at a given point in time. Each commit contains only the incremental
difference from the previous commit, called *delta* in git.

A chain of commits in which all the ancestors are included makes a 
branch. Since every commit is linked to its parent, a branch is
simply a *pointer* to a commit (the full chain of commits can always
be reconstructed from the commit).
In other words, you can think of branches as human friendly labels
for commits. Every time you create a new commit, the pointer of the
current branch advances to the newly created commit.


#### **Tag**

A tag is a human friendly name for a commit but unlike branches, tags
are read-only. Once created, they cannot be modified to point to
a different commit.

Tags are commonly used for labeling commits that constitute 
milestones in the project development timeline, for example a release.


#### **Fork**

A fork is basically a copy of someone else's repository. 
Usually, you cannot create branches or change code in other 
people's repositories, that's why you create your own copy of it. 
This is called `forking`.


#### **Remote**

A git *remote* is a copy of your repository hosted by a git service
provider, e.g. [GitHub](https://github.com). Remotes allow you to
share your commits and branches so that other developers can fetch
them. Technically speaking, remotes are exactly the same as local
repositories, but unlike your local repository, they are reachable 
over the internet.

You can use the commands `git fetch` and `git push` to bring your
local copy of the repository in sync with a remote, by downloading
commits or uploading new commits respectively.


#### **Merging branches**

Merging is the dual operation of creating a new branch. Imaigne you have
branched out a new branch (e.g. _new-feature_) from the some branch
(e.g. _master_), made some improvements and tested them out.
Now you want to incorporate these changes
in the _master_ branch which hosts your main code. The **merge** operation
does exactly that. It takes the changes done in _new-feature_ and applies them
to _master_.

Often git will manage to apply these changes by itself. However, sometimes
if both _new-feature_ and _master_ changed the same part of the code, git
cannot determine by itself which of the two changes should be kept. Such a
case is called _merge conflict_ and you will have to manually select what 
should be kept after the merge.
    
#### **Pull Requests**

If you are working on a secondary branch or if you forked a repository 
and want to submit your changes for integration into the mainstream branch 
or repository, you can open a so-called Pull Request (in short **PR**).

A pull request can be seen as a three-step merge operation between two 
branches where the changes are first _proposed_, then _discussed and adapted_
(if requested), and finally _merged_.


### Common operations

#### **Fork a repository on GitHub**

To fork (creating a copy of a repository, that does not belong to 
you), you simply have to go to the repository's webpage and click 
fork on the upper right corner.


#### **Clone a repository**

Cloning a repository is the act of creating a local copy of a remote
repository. A repo is cloned only at the very beginning, when you
still don't have a local copy of it.

To clone a repository, either copy the HTTPS or SSH link given on 
the repository's webpage. Use the following command to create a local
copy of the remote git repository identified by the given URL.

    git clone [REPOSITORY-URL]
    
This will create a directory in the current working path with 
the same name of the repository and the entire history of commits 
will be downloaded onto your computer.


#### **Create a new branch**

The command for creating a new branch is a little bit 
counter-intuitive, but you will get use to it.
Use the following command to create a new branch:

    git checkout -b [NEW-BRANCH-NAME]
    
This creates a new branch pointing at the same commit your
currently active branch is pointing at. In other words, you will
end up with two branches pointing at the same commit. Note that
after you issue this command, the newly created branch becomes
your active branch.


#### **Working tree**

In git, we use the term *working tree* to indicate all the changes 
that are not committed yet. You can think of it as your workspace.
When you create a new commit, the hash for the current working tree 
is computed and assigned to the new commit together with the changes
since the last commit. The working tree clears as you commit changes.

Remark: You cannot create commits from a clean working tree.

Use the command `git status` to inspect the status of your working
tree.


#### **Create a new commit**

Unlike many git operations, a commit is not created by a single
git command. There are two steps to follow. First, we mark all the 
changes that we want to be part of our new commit, second, we create
the commit. From your working tree, mark changes to include in the 
new commit using the command:
 
    git add [FILE]

The command `git status` will always show you which changes are 
marked to be used for a new commit and which changes are not.
Use the command

    git commit -m "[COMMIT-MESSAGE]"

to create a new commit. Replace `[COMMIT-MESSAGE]` with your
notes about what changes this commit includes.

Note: Do not underestimate the value of good commit messages,
the moment you will go back to your history of commits looking
for a change of interest, having good commit messages will be a 
game changer.


#### **Push changes**

Use the following command to *push* your local changes to the remote
repository so that the two repositories can get in sync.

    git push origin [BRANCH-NAME]


### **Fetch changes**

If you suspect that new changes might be available on the remote
repository, you can use the command 

    git fetch origin [BRANCH-NAME]

to download the new commits available on the remote (if any).
These new changes will be appended to the branch called 
`origin/[BRANCH-NAME]` in your local repository.
If you want to apply them to your current branch, use the command

    git merge origin/[BRANCH-NAME]
    
Use the command `git pull origin/[BRANCH-NAME]` to perform *fetch*
and then *merge*.


#### **Delete branches**

Unlike the vast majority of git commands, git delete does not
work on the current branch. You can delete other branches by running
the command

    git branch -d [BRANCH-NAME]

If you want to delete your current branch, you will need to checkout
another branch first. This prevents ending up with a repository
with no branches.

To propagate the deletion of a branch to the remote repository,
run the command:

    git push origin --delete [BRANCH-NAME]


#### **Open a GitHub Issue**

If you are experiencing issues with any code or content of a repository 
(such as this operating manual you are reading right now), you can submit 
issues. For doing so go to the dashboard of the corresponding repository 
and press the `Issues` tab where you can open a new request.

For example you encounter a bug or a mistake in this operating manual, 
please visit this repository's 
[Issues page](https://github.com/duckietown/docs-opmanual_developer/issues) 
to report an issue.

GitHub Issues are a crucial part of the life cycle of a software product, as
they constitute a feedback loop that goes directly from the end-user to the
product developers. You don't have to be a developer or an expert in software
engineering to open an Issue. 


## Hands on

### Git

It is strongly suggested to all git beginners to follow the 
awesome tutorial 
[Learn Git Branching](https://learngitbranching.js.org/).

Further reading material can be found at the following links:

- [Git Handbook](https://guides.github.com/introduction/git-handbook/)
- [Basic Branching and Merging](https://git-scm.com/book/en/v2/Git-Branching-Basic-Branching-and-Merging)


### GitHub

You can gain access to GitHub by creating an account on 
[github.com](https://github.com/join) (if you don't have one already).

A short GitHub tutorial is available at 
[this link](https://guides.github.com/activities/hello-world/).

It is higly suggested that you setup an SSH key for secure passwordless
access to GitHub by following these steps:

1. [Generate a new SSH key](https://help.github.com/en/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
2. [Add SSH key to your GitHub account](https://help.github.com/en/github/authenticating-to-github/adding-a-new-ssh-key-to-your-github-account).


## Ask the community

If you have any questions about how to use of Git in Duckietown,
join and ask on the Duckietown Slack!

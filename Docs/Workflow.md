# Basic Github workflow

## Overview
1. Clone the repo.
1. Create issue.
1. Create a branch.
1. Make your changes.
1. Create a Pull Request (PR)
1. Get PR approved.
1. Merge PR.

## Prerequisites
A laptop or desktop system with the correct version of WPILib installed. See [build.gradle file, the line for GradleRIO, line 3](../build.gradle). Git installed, a Github account that is in the team organization organization. The Tough Techs VSCode extension installed in VSCode and configured as explained in https://github.com/ToughTechs151/robot-template/blob/main/Docs/VSCode.md See [VSCode.md](VSCode.md) You should be logged into GitHub and have authorized VSCode and its extensions. VSCode will ask you to log in when it needs that authorization, you don't need to do this ahead of time.

## Process Steps
1. Clone repo.

   There are three ways to clone the repo from Github.
   Open the repo on GitHub in a browser.
   Look for the green button that says "Code" and click the dropdown. Select "HTTPS" and the copy symbol to copy the URL.
   In VSCode, create a new window and select "Clone Git Repository" on the Welcome tab. If you don't see it, you can use ctrl-shift-P to bring up the command palette and type
   "git clone", then paste the repo URL you previously copied in the dialog field.
   Select the repository destination. This is not the folder the repo files wil be in, it is the folder where all your VSCode projects are kept.
   Select "Open" when asked if you want to open the new repo.

1. Create issue
   Use the "Github" extension that appears in the activity bar on the far left of the screen. Be sure not to confuse this with the "GitHub Actions" extension. The GitHub panel is divided into two halves, the top is for Push Requests (PR, which we will use at step 14) and the bottom is for Issues. To create a new issue, click the "+" symbol at the top of the Issues panel. You will need to fill in a title and assign it to yourself. You may also include a longer description.

1. Create branch

   At the left corner of the status bar you will find the SCM status, which will include the currently checked out branch. You can checkout a different branch by clicking the name of the current branch, which will open some options at in a dialog at the top of the screen. However when you are working on an issue you are assigned you should create the branch from the Issues panel. You should see the issue you just created under "My Issues". If it is not there, click the refresh button (circular arrow) next to the "+". When you hover over the name of the issue you want to work on, you will see a right pointing arrow, which has hover text "Start working on issue and checkout Topic branch". Use this to create the branch. If at some point it asks if you want to publish the branch, say yes. In the SCM Status area the new branch name will appear. Next to the branch name you will see a circle made from two arrows. Press this circle to send your changes to GitHub, and to pull any other changes from GitHub. In FRC we want to send out changes to GitHub as often as possible so you don't lose your work, and others can help you. At the end of every programming session, you should commit any outstanding changes and sync with GitHub as the last action.

1. Make changes
1. Reformat

   We require that the formatting of the Java program follow the standard style. The build will fail if it does not. To make this easier we provide a plugin called "Spotless" that will correct the formatting. To make running it even easier still, you will find that there is a "Reformat" button at the bottom of the VSCode screen in the status line.

1. Build
1. Check results
1. Commit changes

   In the real world you probably would not commit after every build, instead waiting until the code did what you wanted at this stage. However, in FRC we will be working with other people and we want to be able to see the progress as it is being made. In the commit comment field you should explain why you are making this commit compared to the previous one, rather than what the change is.
1. Test
1. Go to step 4 if there are more changes needed.
1. Pull latest, merge
1. Build
1. Check results
1. Create PR
1. Check CI build
1. Get PR approval
1. Merge PR
1. Close PR
1. Delete branch?

# Sonarcloud setup

Code analysis can be done offline using Sonarlint, but it uses a limited set rules. To get a full analysis requires online analysis with Sonarcloud. Sonarcloud is free for open source projects, so you will either have to use a public repository or pay for your Sonarcloud account. You can create your Sonarcloud account using your Github account.

## Create your Sonarcloud account

Create your account at [the Sonarcloud sign up page](https://www.sonarsource.com/products/sonarcloud/signup/)

## Create a Sonarcloud Organization

If you are going to collaborate with other team members, you will need to create an organization. If you already have an ogranization set up on Github, you can import it to
Sonarcloud. This will automatically handle connecting other users to the organization, if they create their own accounts by linking their Github accounts, but is isn't necessary to have more than one user to administer the projects.

Once you have created your account, click on "Analyze new project", this will take you through a step-by-step tutorial, including how to import your Github organization.
If all of your work will be with public repositories then you can choose the free plan.

## Import your repository

You can now import the repository for your project. Sonarcloud will present a list of repositories, you can select any or all of them. if you are forking this template, select your forked template repository.

## Update the README.md file

The README.md file have links to the corresponding Sonarcloud project so it can display badges. You will need to update the Sonarcloud.io project ID for your project in all of the links.

## Set up CI-Analysis

From your Sonarcloud main projects page, click on the project for your new repository TODO

You will need to update the build.gradle and main.yml files to point to your project.

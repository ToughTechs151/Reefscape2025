# Cloning This Template

## Click "Use this template"

## Create new SonarQube project for the new repo

### Add repo to list of Projects on SonarQube
* Log into SonarQube at https://sonarcloud.io
* press the "+" on your home page and select "analyze new project" to create a new project
* Choose your organization
* Choose you repo. Select it with a checkmark if it is there. if not then:
  * Click "GitHub app configuration" at the bottom of the page.
  * Click SonarCloud - Configure.
  * Pull down menu that says "Select repositories"
  * Select your new repository.
  * Click "Save"
  * Close Previous SonarCloud browser tab, and start this section over, starting from "Log into sonarQube"
* Press setup, select "Previous version", click "Create project"
* On the new project page, under Administration, Analysis Method, turn off automatic analysis.
* Click "With GitHub Actions"
* Create secret name SONAR_TOKEN in your repo. Follow directions under "1 Create GitHub Secret"
* Under "2 Create or Update a build file", select Gradle and Groovy.
* Update the build.gradle and .github/workflows/main.yml as directed. Don't worry about the file name being different.

## Things to change in the Repo before first use

- Repo name in README.md. File find and replace "robot-template" to new name.
- Team name in settings.json
  - Change the line with "connectionID" to your SonarQube organization name.
  - Change the line with "projectKey" to the project key from SonarQube for this project.
  - Change the line for "roborio.hostname" to match your team number. Only change the number.
  - Change the line for "sonarcloud.orgname" to your SonarCloud organization name.
  - Change the line for "sonarcloud.projectname" to this project's name.
- Sonarqube account/project in README.md
- sonarqube account/project in build.gradle
  - Change the line that says "sonar.projectKey".
  - Change the line that says "sonar.organization".

The easiest way to do all the above is to use the "Replace in files" item in the "Edit" menu. If your SonarQube organization matches your GitHub organization, then you can simply do a replace of "ToughTech151" with your organizations name. Replace "robot-template" with your Github Repo name. Be sure to do both with "Match case" turned on. You will still need to replace the lower-case version, one in   The only place that you do not want to change robot-template to your repo name is in the file .github/workflows/template-sync.yml, if you want to be able to pull template changes.

## Configure Repo settings

Watch this space.

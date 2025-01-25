
The latest version of the Google style XML file is available at
https://github.com/checkstyle/checkstyle/blob/master/src/main/resources/google_checks.xml

If the version of Checkstyle being used by Checkstyle for Java does not match the version of the style XML used then the extension may fail to initialize, or it may produce strange checkstyle errors. The checkstyle file should be updated to the latest periodically. If it fails to initialize, then the version of Checkstyle used should be updated.  Set the checkstyle versions to the latest supported version by opening the command palette, typing Checkstyle and looking for :Set the Checkstyle Version".  Select "All Supported versions" and choose the latest.

When updating the XML remember to update the lines that have been changed. Since XML files are formatted by Spotless, you should reformat prior to looking for the changed lines. The lines that have been changed are for MemberName and allowedAbbreviations.

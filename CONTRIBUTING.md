# Contributing Practices
## Branching Practices
The `main` branch should be treated as the branch with the last known working code (like for example, the code at the last competition which was known to work).

Branches containing individual features (like the rewrite of our drivetrain code) should branch off `main` for the most part, with the exception of feature branches that require code from other feature branches (like a feature branch that uses the newly written drivetrain code, in this case).

Before opening a pull request, make sure that you've tested your code thoroughly in our simulation to make sure that it works in our simulation, so glaringly obvious errors won't have to be addressed in the pull request reviews and reviews can be focused on enforcing proper code practices.

Before merging a pull request to `main`, make sure that the code has been thoroughly tested on the **robot**. Again, `main` should always have the last known working code.

Releases should be made off the `main` branch for the last known working code, so even if faulty code is pushed to `main` somehow, the last known working code is saved in a release (like the code at the end of a competition that we know works).

## Before you push your code
Make sure to always simulate your code using `./gradlew simulateJava` to thoroughly test the new code you've pushed and always make sure to clean up your code with Spotless Apply so we know that our code follows Kotlin code practices.

You can clean up your code to follow Kotlin practices with `./gradlew spotlessApply`. If you don't clean up your code with Spotless Apply, your code won't be able to be merged because the workflows will fail, enforcing that your code follows the Kotlin code practices.

When committing your code, use descriptive messages. If you're pushing code that adds a new state to superstructure which flashes the LEDs blue when a certain task occurs, don't write something like `"superstructure changed"`. Instead, write something more descriptive like `"Added new superstructure state for flashing LEDs blue when [task]"`.

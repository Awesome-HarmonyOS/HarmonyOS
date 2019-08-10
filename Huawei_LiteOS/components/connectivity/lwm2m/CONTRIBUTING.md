# Guide to contributing

Please read this if you intend to contribute to the project.

## Legal stuff

Apologies in advance for the extra work required here - this is necessary to comply with the Eclipse Foundation's
strict IP policy.

Please also read [this](http://wiki.eclipse.org/Development_Resources/Contributing_via_Git)

In order for any contributions to be accepted you MUST do the following things.

* Sign the [Eclipse Foundation Contributor License Agreement](http://www.eclipse.org/legal/CLA.php).
To sign the Eclipse CLA you need to:

  * Obtain an Eclipse Foundation userid. Anyone who currently uses Eclipse Bugzilla or Gerrit systems already has one of those.
If you don’t, you need to [register](https://dev.eclipse.org/site_login/createaccount.php).

  * Login into the [projects portal](https://projects.eclipse.org/), select “My Account”, and then the “Contributor License Agreement” tab.

* Add your github username in your Eclipse Foundation account settings. Log in it to Eclipse and go to account settings.

* "Sign-off" your commits

Every commit you make in your patch or pull request MUST be "signed off".

You do this by adding the `-s` flag when you make the commit(s), e.g.

    git commit -s -m "Shave the yak some more"

## Making your changes

* Fork the repository on GitHub
* Create a new branch for your changes
* Make your changes
* Make sure you include tests
* Make sure the test suite passes after your changes
* Commit your changes into that branch
* Use descriptive and meaningful commit messages
* If you have a lot of commits squash them into a single commit
* Make sure you use the `-s` flag when committing as explained above.
* Push your changes to your branch in your forked repository

## Submitting the changes

Submit a pull request via the normal GitHub UI.

## After submitting

* Do not use your branch for any other development, otherwise further changes that you make will be visible in the PR.



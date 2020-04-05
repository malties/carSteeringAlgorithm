# group_06

## How to clone and install
In order to be able to clone the repository, an SSH key is necessary. The steps to clone and install are:
   1. Navigate to your .ssh folder in the root directory.
   2. Run ssh-keygen with the preferred encryption type (RSA / ED25519).
   3. Copy your public key that has been generated and paste it into Gitlab under Your SSH Keys.
   4. Navigate into a folder where you intend to clone the repository. Do: <br>
        *$ git clone git@git.chalmers.se:courses/dit638/students/group_06.git*
   5. Configure your Git authentication using <br>
        *$ git config user.email " ... "* <br>
    and <br>
        *$ git config user.name " ... "*   
   6. To run the program, follow the commands:
        <br>
        *$ docker load < miruna-example.tar.gz*
        <br>
        *$ docker run --rm miruna/example:latest 42*

        This should print 0, meaning 42 is not a prime number.

## Working conventions and policies
   1. Features should be added only upon group discussions and agreement to improve the system.
   2. Features should be present in the working Gitlab boards.
   3. Features should be broken down into tasks - tasks can be taken on by one or multiple interested members.
   4. Tasks can be worked on individually, but another member can join in to work together (to achieve pair programming).
   5. Depending on the importance of the feature and how early it was implemented, some solutions include: reverting to a previous working version, waiting before merging a feature branch into the master branch in order to assess if the feature can be integrated, or using different branches for ideas that might not be feasible to avoid clumping the master branch.
   6. Commit messages should include a label in the beginning of the message (feature, bugfix, etc.).
   7. Commit messages should be self-explanatory, directly naming what the commit does to the product.
   8. Commits should have a label, a title and a description (for extra details that might not be necessary in the title).
   9. Commit title should not contain more than 10 words.
   10. Commit titles should start with a verb ('added', 'created', 'removed', etc.), followed by the rest of the sentence.


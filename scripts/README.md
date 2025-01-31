These files are for the development of the tests. They overlap mostly with industrial_ci, but are easier to customize

The tests are run by the GitHub action on merge and pull request, acting as CI.
They can also be run locally with act (see run_tests.sh in this folder).

- start a docker container with start_docker.sh
- prepare the workspace with setup_workspace.sh
# template package of ACELAB autonomous driving

![dependency diagram](doc/template.drawio.png)

## Tutorials

### 1. pre-commit

- install pre-commit
     ```
     python -m pip install pre-commit cpplint
     pre-commit install
     ```

### 2. changelog
   
  - Add commit to CHANGELOG.md
    ```bash
    git cliff --unreleased --tag v1.0.1 --prepend CHANGELOG.md
    ```

### 3. test
- run test
  ```bash
  catkin build your_package_name --make-args tests
  catkin run_tests
  ```
- check result
  ```bash
  catkin_test_results
  ```
- test option
  >disable test
  ```bash
  catkin build your_package_name -DCATKIN_ENABLE_TESTING=OFF
  ```
  >enable test
  ```bash
  catkin build your_package_name -DCATKIN_ENABLE_TESTING=ON
  ```
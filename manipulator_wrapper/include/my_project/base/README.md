# base

## Parameter manager

```yaml
runtime:
    models:
        - name: "model1"
          type: "type1"
          parameters:
              - name: "param1"
                value: 1
              - name: "param2"
                value: 2
        - name: "model2"
          type: "type2"
          parameters:
              - name: "param1"
                value: 1
              - name: "param2"
                value: 2
system:
    verbosity: "info"
    period: 0.1
```

```c++
// 1. using Node
auto node = parameter_manager_->getNode({"runtime", "models"});
for (const auto& model : models){
    // get parameters...
}

// 2. get value
auto val = parameter_manager_->get<std::string>({"system", "verbosity"});

// 3. get value with default value
auto period = parameter_manager_->get<std::string>({"system", "period"}, 0.3);
```

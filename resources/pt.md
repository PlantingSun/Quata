## How policy works?

### Step1: policy and control_type

  Input: observation

  Output: action (normalized position)

### Step2: action_scale

  Input: action (normalized position)

  Output: target (position)

### Step3: PD (stiffness and damping)

  Input: target (position)

  Output: torque/force

### Step4: step

  Input: torque/force

  Output: observation

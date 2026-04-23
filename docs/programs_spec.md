/experimental/cells/{cell}/programs:
get:
tags: - Program
summary: List programs
description: |
**Required permissions:** `can_operate_programs` - Execute and monitor programs
\_\_\_

        <!-- theme: danger -->

        > **Experimental**

        List details of all existing programs.
      operationId: listPrograms
      x-scope: can_operate_programs
      parameters:
        - $ref: '#/components/parameters/Cell'
      responses:
        '200':
          description: Successful Response
          content:
            application/json:
              schema:
                title: Response of list programs
                type: array
                items:
                  $ref: '#/components/schemas/Program'
              examples:
                two_programs:
                  summary: Example with two programs
                  description: Sample response showing two different programs
                  value:
                    - program: pick_and_place
                      name: Pick and Place Operation
                      description: A program that picks up objects from a conveyor and places them in designated locations
                      app: app1
                    - program: welding_sequence
                      name: Automated Welding Sequence
                      description: Complex welding program with multiple passes and quality checks
                      app: app2
        '500':
          description: Internal server error

/experimental/cells/{cell}/programs/{program}:
get:
tags: - Program
summary: Get program
description: |
**Required permissions:** `can_operate_programs` - Execute and monitor programs
\_\_\_

        <!-- theme: danger -->

        > **Experimental**

        Get details of a program.
      operationId: getProgram
      x-scope: can_operate_programs
      parameters:
        - $ref: '#/components/parameters/Cell'
        - $ref: '#/components/parameters/Program'
      responses:
        '200':
          description: Successful Response
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Program'
              example:
                program: pick_and_place
                name: Pick and Place Demo
                description: A demo program for pick and place operations
                app: app1
        '500':
          description: Internal server error

/experimental/cells/{cell}/programs/{program}/start:
post:
tags: - Program
summary: Start the program
description: |
**Required permissions:** `can_operate_programs` - Execute and monitor programs
\_\_\_

        <!-- theme: danger -->

        > **Experimental**

        This endpoint starts a new program execution.

        The program will be executed asynchronously.
      operationId: startProgram
      x-scope: can_operate_programs
      parameters:
        - $ref: '#/components/parameters/Cell'
        - $ref: '#/components/parameters/Program'
      requestBody:
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/ProgramStartRequest'
        required: true
      responses:
        '200':
          description: Successful Response
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ProgramRun'
              examples:
                minimal:
                  summary: Minimal example
                  value:
                    run: run_125
                    program: app1.pick_and_place
                    state: PREPARING
                    start_time: '2024-01-15T12:00:00Z'
        '400':
          description: Either a syntax or a runtime error
        '404':
          description: Not found
        '406':
          description: A program run is already running
        '422':
          description: Validation Error
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/HTTPValidationError'
        '452':
          description: |
            Robot is not able to perform the motion due to hard or soft restrictions.
            This can be due to a variety of reasons:

            - The robot is too close to a singularity,
            - The robot is too close to a joint limit,
            - Robot operating mode could not be set to the desired mode,
            - An operating mode change occurred during the motion.

            In the case of an error, the full error will be returned in the response body.
        '500':
          description: Internal server error

/experimental/cells/{cell}/programs/{program}/stop:
post:
tags: - Program
summary: Stop program run
description: |
**Required permissions:** `can_operate_programs` - Execute and monitor programs
\_\_\_

        <!-- theme: danger -->

        > **Experimental**

        Stop a specific program run.
      operationId: stopProgram
      x-scope: can_operate_programs
      parameters:
        - $ref: '#/components/parameters/Cell'
        - $ref: '#/components/parameters/Program'
      responses:
        '204':
          description: Successful Response. Will also be returned if the run is not running.
        '404':
          description: Not found
        '500':
          description: Internal server error

/experimental/cloud/connect:
post:
operationId: connectToNovaCloud
x-scope: can_manage_cloud_connection
summary: Connect to NOVA Cloud
tags: - NOVA Cloud
description: |-
**Required permissions:** `can_manage_cloud_connection` - Manage NOVA Cloud connection
\_\_\_

          <!-- theme: danger -->

          > **Experimental**

          Register this instance with the NOVA Cloud fleet manager and configure the
          local NATS server to establish a leafnode connection. The fleet manager will
          then be able to receive event data from this instance, allowing it to monitor
          the connected robots.

          Establishing the connection can take some time (~30-60 seconds), as the NATS
          server pod in the cluster needs to restart to apply the new configuration.
      parameters:
        - schema:
            type: integer
            minimum: 1
            description: |-

              The maximum time (**in seconds**) spent waiting until the operation is complete.

              If the parameter is set, the request will wait for completion until the specified time is up.
              For POST and PUT requests completion means that all resources are running and usable.
              For DELETE completion means that the deletion process is completed.
          required: false
          description: |-

            The maximum time (**in seconds**) spent waiting until the operation is complete.

            If the parameter is set, the request will wait for completion until the specified time is up.
            For POST and PUT requests completion means that all resources are running and usable.
            For DELETE completion means that the deletion process is completed.
          name: completion_timeout
          in: query
      requestBody:
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/CloudConnectionRequest'
      responses:
        '200':
          description: The instance was successfully connected to NOVA Cloud.
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/CloudRegistrationSuccessResponse'
        '202':
          description: The instance was registered and the connection process has started.
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/CloudRegistrationSuccessResponse'
        '400':
          description: The request parameters failed to validate.
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ZodValidationError'
        '424':
          description: The connection to NOVA Cloud could not be established.
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/CloudConnectionError'

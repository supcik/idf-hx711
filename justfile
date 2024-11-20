PATH:="get_started"
COMPONENT:="hx711"

build:
    idf.py build --project-dir {{COMPONENT}}/examples/{{PATH}}

flash:
    idf.py flash --project-dir {{COMPONENT}}/examples/{{PATH}}

monitor:
    idf.py monitor --project-dir {{COMPONENT}}/examples/{{PATH}}

menuconfig:
    idf.py menuconfig --project-dir {{COMPONENT}}/examples/{{PATH}}

set-target TARGET='esp32s3':
    idf.py set-target --project-dir {{COMPONENT}}/examples/{{PATH}} {{TARGET}}

clean:
    idf.py fullclean --project-dir {{COMPONENT}}/examples/{{PATH}}

save-defconfig:
    idf.py save-defconfig --project-dir {{COMPONENT}}/examples/{{PATH}}

update-settings:
     jq '."idf.buildPath"|="${workspaceFolder}/{{COMPONENT}}/examples/{{PATH}}/build"' ./.vscode/settings.json \
     > ./.vscode/temp.json && mv ./.vscode/temp.json ./.vscode/settings.json

controllers = {};

gamepad_connected_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/gamepad/connected',
    messageType : 'std_msgs/Bool'
});

window.addEventListener("gamepadconnected", function (e) {
    let gp = navigator.getGamepads()[e.gamepad.index];
    document.getElementById("controller_status").innerText = "Controller: " + gp.id;
    console.log("Gamepad connected at index %d: %s. ", gp.index, gp.id, gp);
    controllers[e.gamepad.index] = e.gamepad;
    if(Object.keys(controllers).length > 0) {
        gamepad_connected_message = new ROSLIB.Message({
            data: true
        })
    }

    gamepad_connected_topic.publish(gamepad_connected_message)
});

window.addEventListener("gamepaddisconnected", function (e) {
    document.getElementById("controller_status").innerText = "Controller: None";
    console.log("Gamepad disconnected from index %d: %s",
        e.gamepad.index, e.gamepad.id);
    delete controllers[e.gamepad.index];
    if(Object.keys(controllers).length === 0) {
        gamepad_connected_message = new ROSLIB.Message({
            data: false
        })
    }

    gamepad_connected_topic.publish(gamepad_connected_message)
});

gamepad_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/gamepad/values',
    messageType : 'sensor_msgs/Joy'
});

function reduceDeadzone(x) {
    let deadzone = 0.2;
    if (Math.abs(x) < deadzone) {
        return 0
    } else if(x > 0) {
        return (x - deadzone) / (1 - deadzone)
    } else if(x < 0) {
        return -(-x - deadzone) / (1 - deadzone)
    }
}

function pollGamepad() {
    if(Object.keys(controllers).length === 0) {
        return 0
    }

    let controller = navigator.getGamepads()[0];

    let buttons = [];
    let axes = [];

    for (let i = 0; i < controller.axes.length; i++) {
        axes.push(reduceDeadzone(controller.axes[i]));
    }

    for (let i = 0; i < controller.buttons.length; i++) {
        let b = document.getElementById("button" + String(i));
        let val = controller.buttons[i];
        let pressed = val == 1.0;
        if (typeof (val) == "object") {
            pressed = val.pressed;
            val = val.value;
        }

        if(i === 6 || i === 7) {
            axes.push(val);
            buttons.push(0);
        } else {
            buttons.push(Math.round(val));
        }

        b.style.opacity = 0.2 + val * 0.8;
    }

    let sensitivity = 20;
    document.getElementById("button10").setAttribute("transform", "translate(" + (axes[0] * sensitivity) + ", " + (axes[1] * sensitivity) + ")");
    document.getElementById("button11").setAttribute("transform", "translate(" + (axes[2] * sensitivity) + ", " + (axes[3] * sensitivity) + ")");

    let gamepad_message = new ROSLIB.Message({
        axes: axes,
        buttons: buttons
    });

    gamepad_topic.publish(gamepad_message);
}

setInterval(pollGamepad, 10);
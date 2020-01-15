function power_init(id) {
    var listener = new ROSLIB.Topic({
        ros: ros,
        name: '/battery/cell_voltages',
        messageType: 'std_msgs/Float32MultiArray'
    });

    var total_current_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/battery/total_current',
        messageType: 'std_msgs/Float32'
    });

    var odrive_current_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/battery/odrive_current',
        messageType: 'std_msgs/Float32'
    });

    total_current_listener.subscribe(function(message) {
        console.log(message);
        document.getElementById("total-current").style.height = message.data * 100 / 15 + "%";
    });

    odrive_current_listener.subscribe(function(message) {
        console.log(message);
        document.getElementById("odrive-current").style.height = message.data * 100 / 15 + "%";
    });

    listener.subscribe(function (message) {
        let total_voltage = 0;
        for(let i in message.data) {
            document.getElementById("cell-" + String(i)).innerHTML = message.data[i].toFixed(2) + "v";
            let indicator_color = "green";

            if(message.data[i] < 3) {
                indicator_color = "red";
            } else if(message.data[i] < 3.7) {
                indicator_color = "orange";
            }

            document.getElementById("cell-" + i + "-indicator").style.background = indicator_color;

            total_voltage += message.data[i];
        }

        let percentage = (total_voltage - 4 * 3) * 100 / (message.data.length * 4.2 - 4 * 3);

        document.getElementById("overall-voltage").innerHTML = total_voltage.toFixed(2) + "v (" + percentage.toFixed(1) + "%)";
    });
}
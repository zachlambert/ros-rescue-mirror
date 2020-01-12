function power_init() {
    var listener = new ROSLIB.Topic({
        ros: ros,
        name: '/battery/cell_voltages',
        messageType: 'std_msgs/Float32MultiArray'
    });

    listener.subscribe(function (message) {
        let total_voltage = 0;
        for(let i in message.data) {
            document.getElementById("cell-" + String(i)).innerHTML = message.data[i].toFixed(2) + "v";
            total_voltage += message.data[i];
        }

        let percentage = total_voltage * 100 / (message.data.length * 4.2);

        document.getElementById("overall-voltage").innerHTML = total_voltage.toFixed(2) + "v (" + percentage.toFixed(1) + "%)";
    });
}
function odrive_init() {
    calibrate_odrive = [];
    names = ["drive", "flipper"];

    for (let i = 0; i < 2; i++) {
        calibrate_odrive[i] = [];
        for (let j = 0; j < 2; j++) {
            calibrate_odrive[i][j] = new ROSLIB.Service({
                ros: ros,
                name: '/odrive_' + names[i] + '/axis' + j + '/calibrate',
                serviceType: 'std_srvs/Trigger'
            });
            document.getElementById("odrives_table").innerHTML += "<tr id='odrive_" + i + "_" + j + "'><td>ODrive " + i + " axis " + j + "</td><td><button onclick='start_calibration(" + i + ", " + j + ")'>Calibrate</button></td><td></td></tr>"
        }
    }
}

function start_calibration(i, j) {
    document.getElementById("odrive_" + i + "_" + j).children[2].innerHTML = "Calibrating...";
    calibrate_odrive[i][j].callService(null, function(response) {
        document.getElementById("odrive_" + i + "_" + j).children[2].innerHTML = response.message;
    })
}
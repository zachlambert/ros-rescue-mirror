function odrive_init(id) {
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
            document.getElementById("mod_odrive_" + id).getElementsByClassName("odrives_table")[0].innerHTML += "<tr class='odrive_" + i + "_" + j + "'><td>D" + i + "A" + j + "</td><td></td><td><button onclick='start_calibration(" + i + ", " + j + ", \"" + id + "\")'>Calibrate</button></td></tr>"
        }
    }
}

function start_calibration(i, j, id) {
    document.getElementById("mod_odrive_" + id).getElementsByClassName("odrive_" + i + "_" + j)[0].children[1].innerHTML = "Calibrating...";
    calibrate_odrive[i][j].callService(null, function(response) {
        document.getElementById("mod_odrive_" + id).getElementsByClassName("odrive_" + i + "_" + j)[0].children[1].innerHTML = response.message;
    })
}
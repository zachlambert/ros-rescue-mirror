function co2_init(id) 
{
    var listener = new ROSLIB.Topic({
        ros: ros,
        name: 'pub_co2',
        messageType: 'std_msgs/UInt16'
    });
    
    listener.subscribe(function(message) {
        document.getElementById("ppm").innerHTML = message.data;
        if(message.data<420)
            document.getElementById("ppm").style.color = "blue";
	    else
		    if(450<=message.data)
                document.getElementById("ppm").style.color = "brown";
	   });

}


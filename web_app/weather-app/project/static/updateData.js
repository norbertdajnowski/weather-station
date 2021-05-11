
 var moreStatus = false;
 var interval = 10000;
 var stationNumber = 0;
 
 $( document ).ready( start );

 function start( jQuery ) {  
    //hide extra info fields
    $('.light').hide();
    $('.wind-direction').hide();
    $('.rain-rate').hide();

    //Fast forward first request
    dataUpdate();

    //appends an "active" class to .popup and .popup-content when the "Open" button is clicked
    $("#open").on("click", function() {
        $(".popup-content, .popup-overlay").addClass("active");
    });
  
    //removes the "active" class to .popup and .popup-content when the "Close" button is clicked 
    $("#close, .popup-overlay").on("click", function() {
        $(".popup-content, .popup-overlay").removeClass("active");
    });

    $("#campus-station").on("click", function(){
        stationNumber = 0;
        $(".popup-content, .popup-overlay").removeClass("active");
    });

    $("#sports-station").on("click", function(){
        stationNumber = 1;
        $(".popup-content, .popup-overlay").removeClass("active");
    });
}



 function dataUpdate() {
    try {
        var call_url = '/data_update?stationNumber=' + stationNumber;
        $.getJSON(call_url, function(results){

            //Display weather data
            $('#temperature').text(results["results"][0]["temperature"] + " °C");
            $('#humidity').text(results["results"][0]["humidity"] + " %");
            $('#pressure').text(results["results"][0]["pressure"] + " kP");
            $('#light').text(results["results"][0]["light"] + " V");
            $('#carbonM').text(results["results"][0]["carbon"] + " PPM");
            $('#windS').text(results["results"][0]["windS"] + " km/h");
            $('#windD').text(results["results"][0]["windD"]);
            $('#rainR').text(results["results"][0]["rain"] + " mm");

            //Display meta data
            $('.date-dayname').text(results["results"][1]["day"]);
            $('.location').text(results["results"][1]["stationName"]);
            $('.date-day').text(results["results"][1]["timedate"]);
            $('.weather-desc').text(results["results"][1]["weatherCondition"]);
        })
    } catch (err){
        alert("Error encountered: " + err);
    }
 }

 //Show/hide extra weather information
 function infoDisplay() {
    if (moreStatus == false){
       $('.light').show(1000);
       $('.wind-direction').show(1000);
       $('.rain-rate').show(1000);
       moreStatus = true;
    }
    else{
        $('.light').hide(1000);
        $('.wind-direction').hide(1000);
        $('.rain-rate').hide(1000);
        moreStatus = false;
    }

 }

 //Calls data update function after every interval
 setInterval(dataUpdate, interval)
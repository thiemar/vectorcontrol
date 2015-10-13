var seriesColors = [
    /* First index is series number; second index is amplitude number */
    ["#393b79", "#6b6ecf", "#9c9ede"],
    ["#637939", "#8c8252", "#cedb9c"],
    ["#8c6d31", "#bd9e39", "#e7cb94"],
    ["#84ec39", "#d6616b", "#e7969c"],
    ["#7b4173", "#ce6dbd", "#de9ed6"]
];


function createResponseChart(chartId, width, height, margin) {
    var result = { chart: null, x: null, y0: null, y1: null },
        xAxis, yAxis0, yAxis1;

    result.x = d3.scale.log()
        .base(2)
        .range([0, width])
        .domain([1.0, 16.0]);

    result.y0 = d3.scale.linear()
        .range([height, 0.0])
        .domain([0.125, 1.25]);

    result.y1 = d3.scale.linear()
        .range([height, 0.0])
        .domain([-210.0, 60.0]);

    xAxis = d3.svg.axis()
        .scale(result.x)
        .orient("bottom")
        .tickValues([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16])
        .tickSize(-height, 0, 0)
        .tickFormat(function (d) { return "" + d; });
    yAxis0 = d3.svg.axis()
        .scale(result.y0)
        .orient("left")
        .tickValues([1.25, 1.125, 1.0, 0.875, 0.75, 0.625, 0.5, 0.375, 0.25, 0.125])
        .tickSize(-width, 0, 0);
    yAxis1 = d3.svg.axis()
        .scale(result.y1)
        .orient("right")
        .tickValues([60, 30, 0, -30, -60, -90, -120, -150, -180, -210])
        .tickSize(0, 0, 0);

    result.chart = d3.select("#" + chartId)
        .attr("width", width + margin.left + margin.right)
        .attr("height", height + margin.top + margin.bottom)
        .append("g")
            .attr("transform", "translate(" + margin.left + "," + margin.top + ")");

    result.chart.append("g")
        .attr("class", "x axis")
        .attr("transform", "translate(0, " + height + ")")
        .call(xAxis)
    .append("text")
        .attr("x", width / 2)
        .attr("y", 30)
        .style("text-anchor", "middle")
        .text("Frequency (Hz)");

    result.chart.append("g")
        .attr("class", "y0 axis")
        .call(yAxis0)
    .append("text")
        .attr("x", -height / 2)
        .attr("y", -30)
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .text("— Thrust amplitude ratio");

    result.chart.append("g")
        .attr("class", "y1 axis")
        .attr("transform", "translate(" + width + ", 0)")
        .call(yAxis1)
    .append("text")
        .attr("x", -height / 2)
        .attr("y", 40)
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .text("- - Phase (°)");

    result.chart.append("clipPath")
        .attr("id", "clip")
    .append("rect")
        .attr("width", width)
        .attr("height", height);

    for (var i = 0; i < seriesColors.length; i++) {
        for (var j = 0; j < seriesColors[i].length; j++) {
            result.chart.append("path")
                .attr("class", "magnitude response-magnitude-" + i + "-" + j)
                .attr("clip-path", "url(#clip)")
                .attr("stroke", seriesColors[i][j]);
            result.chart.append("path")
                .attr("class", "phase response-phase-" + i + "-" + j)
                .attr("clip-path", "url(#clip)")
                .attr("stroke", seriesColors[i][j]);
        }
    }

    return result;
}


function plotResponse(chart, series, amplitudeIndex, data) {
    var magnitudeLine, phaseLine;

    magnitudeLine = d3.svg.line().interpolate("basis")
        .x(function(d) { return chart.x(d.f_hz); })
        .y(function(d) { return chart.y0(d.est_magnitude); });
    chart.chart.select(".response-magnitude-" + series + "-" + amplitudeIndex)
         .datum(data)
         .attr("d", magnitudeLine);

    phaseLine = d3.svg.line().interpolate("basis")
        .x(function(d) { return chart.x(d.f_hz); })
        .y(function(d) { return chart.y1(d.est_phase_rad / Math.PI * 180.0); });
    chart.chart.select(".response-phase-" + series + "-" + amplitudeIndex)
         .datum(data)
         .attr("d", phaseLine);
}


function createThrustPowerChart(chartId, width, height, margin) {
    var result = { chart: null, x: null, y: null },
        xAxis, yAxis;

    result.x = d3.scale.linear()
        .range([0, width])
        .domain([0.0, 10.0]);

    result.y = d3.scale.linear()
        .range([height, 0.0])
        .domain([0.0, 200.0]);

    xAxis = d3.svg.axis()
        .scale(result.x)
        .orient("bottom")
        .tickValues([0.0,1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0])
        .tickSize(-height, 0, 0);
    yAxis = d3.svg.axis()
        .scale(result.y)
        .orient("left")
        .tickValues([0.0,20.0,40.0,60.0,80.0,100.0,120.0,140.0,160.0,180.0,200.0])
        .tickSize(-width, 0, 0);

    result.chart = d3.select("#" + chartId)
        .attr("width", width + margin.left + margin.right)
        .attr("height", height + margin.top + margin.bottom)
        .append("g")
            .attr("transform", "translate(" + margin.left + "," + margin.top + ")");

    result.chart.append("g")
        .attr("class", "x axis")
        .attr("transform", "translate(0, " + height + ")")
        .call(xAxis)
    .append("text")
        .attr("x", width / 2)
        .attr("y", 30)
        .style("text-anchor", "middle")
        .text("Thrust (N)");

    result.chart.append("g")
        .attr("class", "y axis")
        .call(yAxis)
    .append("text")
        .attr("x", -height / 2)
        .attr("y", -30)
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .text("— Power (W)");

    result.chart.append("clipPath")
        .attr("id", "clip")
    .append("rect")
        .attr("width", width)
        .attr("height", height);

    for (var i = 0; i < seriesColors.length; i++) {
        result.chart.append("path")
            .attr("class", "power thrust-power-" + i)
            .attr("clip-path", "url(#clip)")
            .attr("stroke", seriesColors[i][0]);
    }

    return result;
}


function plotThrustPower(chart, series, data) {
    var line;

    line = d3.svg.line().interpolate("basis")
        .x(function(d) { return chart.x(d.est_thrust_n); })
        .y(function(d) { return chart.y(d.power_w); });

    chart.chart.select(".thrust-power-" + series)
         .datum(data)
         .attr("d", line);
}


var thrustPowerChart, responseChart, ws;


function setup() {
    thrustPowerChart = createThrustPowerChart(
        "thrust-power-chart", 800, 300,
        {top: 20, right: 50, bottom: 40, left: 50});
    responseChart = createResponseChart(
        "response-chart", 800, 300,
        {top: 10, right: 50, bottom: 40, left: 50});

    document.getElementById("content").addEventListener("click", function(event) {
        if (event.target.id == "bandwidth-test") {
            ws.send(JSON.stringify({"test": "response"}));
        } else if (event.target.id == "power-test") {
            ws.send(JSON.stringify({"test": "power"}));
        }
    });
}


function connect() {
    var ws_timeout, lastUpdate;
    ws = new WebSocket("ws://" + window.location.host + "/ws");
    lastUpdate = (new Date()).valueOf();

    ws.onmessage = function(event) {
        var message;
        message =  JSON.parse(event.data);
        if (message.test == "response") {
            plotResponse(responseChart, 0, 0, message.data["1.0"] || []);
            plotResponse(responseChart, 0, 1, message.data["2.0"] || []);
            plotResponse(responseChart, 0, 2, message.data["5.0"] || []);
        } else if (message.test == "power") {
            plotThrustPower(thrustPowerChart, 0, message.data || []);
        }
    }

    /* Send an initial message to trigger the initial fetch */
    ws.onopen = function(event) {
        ws.send("{}");
    }


    // ws.onmessage({data: '{"test":"response","data":{"5.0":[{"est_magnitude":1.007902391224135,"est_phase_rad":-0.02372737174287831,"meas_magnitude":0,"meas_phase_rad":-3.1069025244586403,"f_hz":1,"relative_amplitude":0.5},{"est_magnitude":1.0108269034490436,"est_phase_rad":-0.014237350869746646,"meas_magnitude":0,"meas_phase_rad":-3.0629848397617825,"f_hz":1.189207115002721,"relative_amplitude":0.5},{"est_magnitude":1.0152663709169782,"est_phase_rad":-0.03664422589260985,"meas_magnitude":0,"meas_phase_rad":-3.09005524262496,"f_hz":1.4142135623730951,"relative_amplitude":0.5},{"est_magnitude":1.0214547438829538,"est_phase_rad":-0.011877701050640344,"meas_magnitude":0,"meas_phase_rad":-3.0334793898442642,"f_hz":1.681792830507429,"relative_amplitude":0.5},{"est_magnitude":1.0303631972578546,"est_phase_rad":-0.05234483941925472,"meas_magnitude":0,"meas_phase_rad":-3.070304428541057,"f_hz":2,"relative_amplitude":0.5},{"est_magnitude":1.0418331855549043,"est_phase_rad":-0.03962656008915921,"meas_magnitude":0,"meas_phase_rad":-2.9901493432901214,"f_hz":2.378414230005442,"relative_amplitude":0.5},{"est_magnitude":1.0582204626706204,"est_phase_rad":-0.0525071565872266,"meas_magnitude":0,"meas_phase_rad":-2.92117650430744,"f_hz":2.8284271247461903,"relative_amplitude":0.5},{"est_magnitude":1.082671951219424,"est_phase_rad":-0.10996804843996166,"meas_magnitude":0,"meas_phase_rad":-2.96961308316532,"f_hz":3.363585661014858,"relative_amplitude":0.5},{"est_magnitude":1.1128433236316275,"est_phase_rad":-0.1674874717044328,"meas_magnitude":0,"meas_phase_rad":-3.000080919008164,"f_hz":4,"relative_amplitude":0.5},{"est_magnitude":1.1077002231765853,"est_phase_rad":-0.3329895821129272,"meas_magnitude":0,"meas_phase_rad":-2.8589062683707573,"f_hz":4.756828460010884,"relative_amplitude":0.5},{"est_magnitude":0.9681580777756312,"est_phase_rad":-0.7033136101477875,"meas_magnitude":0,"meas_phase_rad":-2.7456881450268895,"f_hz":5.656854249492381,"relative_amplitude":0.5},{"est_magnitude":0.8117812467808965,"est_phase_rad":-0.9654787694634037,"meas_magnitude":0,"meas_phase_rad":-2.559679546452111,"f_hz":6.727171322029716,"relative_amplitude":0.5},{"est_magnitude":0.6906533602125896,"est_phase_rad":-1.3329136890156537,"meas_magnitude":0,"meas_phase_rad":-2.861431215889641,"f_hz":8,"relative_amplitude":0.5},{"est_magnitude":0.568794306821747,"est_phase_rad":-1.3670870657794927,"meas_magnitude":0,"meas_phase_rad":-2.4815494945606447,"f_hz":9.513656920021768,"relative_amplitude":0.5},{"est_magnitude":0.4759472815091515,"est_phase_rad":-1.5218158264857613,"meas_magnitude":0,"meas_phase_rad":-2.4100427709811867,"f_hz":11.313708498984761,"relative_amplitude":0.5},{"est_magnitude":0.3878151012282894,"est_phase_rad":-1.5787602944064756,"meas_magnitude":0,"meas_phase_rad":-2.1461880786051544,"f_hz":13.454342644059432,"relative_amplitude":0.5},{"est_magnitude":0.33186320435423,"est_phase_rad":-1.844797021626691,"meas_magnitude":0,"meas_phase_rad":-2.5804080701539984,"f_hz":16,"relative_amplitude":0.5}],"2.0":[{"est_magnitude":1.007854511688104,"est_phase_rad":-0.005486376344061128,"meas_magnitude":0,"meas_phase_rad":-3.078835157732041,"f_hz":1,"relative_amplitude":0.2},{"est_magnitude":1.0112739618862276,"est_phase_rad":-0.02438611841970806,"meas_magnitude":0,"meas_phase_rad":-3.0861927638638345,"f_hz":1.189207115002721,"relative_amplitude":0.2},{"est_magnitude":1.0158391542337188,"est_phase_rad":-0.035220264767719556,"meas_magnitude":0,"meas_phase_rad":-3.0899278799821657,"f_hz":1.4142135623730951,"relative_amplitude":0.2},{"est_magnitude":1.0219864648651902,"est_phase_rad":-0.011478153955120085,"meas_magnitude":0,"meas_phase_rad":-3.0335154881379927,"f_hz":1.681792830507429,"relative_amplitude":0.2},{"est_magnitude":1.0317580132233573,"est_phase_rad":-0.05394204917999845,"meas_magnitude":0,"meas_phase_rad":-3.068927074151732,"f_hz":2,"relative_amplitude":0.2},{"est_magnitude":1.0419703497255046,"est_phase_rad":-0.05213566245809487,"meas_magnitude":0,"meas_phase_rad":-3.022916509842055,"f_hz":2.378414230005442,"relative_amplitude":0.2},{"est_magnitude":1.0593305602031973,"est_phase_rad":-0.08578732601977235,"meas_magnitude":0,"meas_phase_rad":-2.9988629781775127,"f_hz":2.8284271247461903,"relative_amplitude":0.2},{"est_magnitude":1.0809369260046435,"est_phase_rad":-0.05348853884126381,"meas_magnitude":0,"meas_phase_rad":-2.8764696917917223,"f_hz":3.363585661014858,"relative_amplitude":0.2},{"est_magnitude":1.1119358920353142,"est_phase_rad":-0.10319498965439555,"meas_magnitude":0,"meas_phase_rad":-2.896289538226069,"f_hz":4,"relative_amplitude":0.2},{"est_magnitude":1.1476298812392352,"est_phase_rad":-0.11208144657632158,"meas_magnitude":0,"meas_phase_rad":-2.765018193527801,"f_hz":4.756828460010884,"relative_amplitude":0.2},{"est_magnitude":1.197361503965179,"est_phase_rad":-0.27966072175789547,"meas_magnitude":0,"meas_phase_rad":-2.905813956477457,"f_hz":5.656854249492381,"relative_amplitude":0.2},{"est_magnitude":1.2339853980186704,"est_phase_rad":-0.2829642126961023,"meas_magnitude":0,"meas_phase_rad":-2.573192189571357,"f_hz":6.727171322029716,"relative_amplitude":0.2},{"est_magnitude":1.272994754413194,"est_phase_rad":-0.4194439842867079,"meas_magnitude":0,"meas_phase_rad":-2.6513882791460346,"f_hz":8,"relative_amplitude":0.2},{"est_magnitude":1.247190464592329,"est_phase_rad":-0.7484870749103352,"meas_magnitude":0,"meas_phase_rad":-2.7229371369743274,"f_hz":9.513656920021768,"relative_amplitude":0.2},{"est_magnitude":1.1123439638948527,"est_phase_rad":-1.0585978995105365,"meas_magnitude":0,"meas_phase_rad":-2.708787827576024,"f_hz":11.313708498984761,"relative_amplitude":0.2},{"est_magnitude":0.9208640841885137,"est_phase_rad":-1.0683479840136414,"meas_magnitude":0,"meas_phase_rad":-2.1471290117741737,"f_hz":13.454342644059432,"relative_amplitude":0.2},{"est_magnitude":0.7769678447131404,"est_phase_rad":-1.2359600495016472,"meas_magnitude":0,"meas_phase_rad":-2.137076701431782,"f_hz":16,"relative_amplitude":0.2}],"1.0":[{"est_magnitude":1.0076528708991594,"est_phase_rad":-0.008288215006990285,"meas_magnitude":0,"meas_phase_rad":-3.07930273369423,"f_hz":1,"relative_amplitude":0.1},{"est_magnitude":1.011033144226079,"est_phase_rad":-0.029643037655633364,"meas_magnitude":0,"meas_phase_rad":-3.095044795119417,"f_hz":1.189207115002721,"relative_amplitude":0.1},{"est_magnitude":1.015318406970147,"est_phase_rad":-0.01687838348344739,"meas_magnitude":0,"meas_phase_rad":-3.0521516577345653,"f_hz":1.4142135623730951,"relative_amplitude":0.1},{"est_magnitude":1.0222649998341236,"est_phase_rad":-0.04427911677699692,"meas_magnitude":0,"meas_phase_rad":-3.0790113889869746,"f_hz":1.681792830507429,"relative_amplitude":0.1},{"est_magnitude":1.0311287414872885,"est_phase_rad":-0.011854895603601978,"meas_magnitude":0,"meas_phase_rad":-3.0159289474462625,"f_hz":2,"relative_amplitude":0.1},{"est_magnitude":1.0428851423504537,"est_phase_rad":-0.033027932911236846,"meas_magnitude":0,"meas_phase_rad":-2.98827260669989,"f_hz":2.378414230005442,"relative_amplitude":0.1},{"est_magnitude":1.0595041601054986,"est_phase_rad":-0.026477486258099425,"meas_magnitude":0,"meas_phase_rad":-2.9191518965567456,"f_hz":2.8284271247461903,"relative_amplitude":0.1},{"est_magnitude":1.0821716841984468,"est_phase_rad":-0.11123131140362386,"meas_magnitude":0,"meas_phase_rad":-2.9739789062192683,"f_hz":3.363585661014858,"relative_amplitude":0.1},{"est_magnitude":1.1105799584430016,"est_phase_rad":-0.08778124996748415,"meas_magnitude":0,"meas_phase_rad":-2.8933262266822335,"f_hz":4,"relative_amplitude":0.1},{"est_magnitude":1.1488225917505885,"est_phase_rad":-0.19973803982665217,"meas_magnitude":0,"meas_phase_rad":-2.899274379656952,"f_hz":4.756828460010884,"relative_amplitude":0.1},{"est_magnitude":1.1920932184028754,"est_phase_rad":-0.2707701058783223,"meas_magnitude":0,"meas_phase_rad":-2.900048655221661,"f_hz":5.656854249492381,"relative_amplitude":0.1},{"est_magnitude":1.2286027115782807,"est_phase_rad":-0.23565924178749142,"meas_magnitude":0,"meas_phase_rad":-2.581074018928913,"f_hz":6.727171322029716,"relative_amplitude":0.1},{"est_magnitude":1.274528921133977,"est_phase_rad":-0.3456815390842567,"meas_magnitude":0,"meas_phase_rad":-2.640628436724721,"f_hz":8,"relative_amplitude":0.1},{"est_magnitude":1.2695417092153596,"est_phase_rad":-0.5830646927939565,"meas_magnitude":0,"meas_phase_rad":-2.4909931577354394,"f_hz":9.513656920021768,"relative_amplitude":0.1},{"est_magnitude":1.2425354582330699,"est_phase_rad":-0.9235175065684289,"meas_magnitude":0,"meas_phase_rad":-2.713468951607238,"f_hz":11.313708498984761,"relative_amplitude":0.1},{"est_magnitude":1.1087355368660885,"est_phase_rad":-0.8352739351739151,"meas_magnitude":0,"meas_phase_rad":-2.164174036006885,"f_hz":13.454342644059432,"relative_amplitude":0.1},{"est_magnitude":0.9946901679254563,"est_phase_rad":-1.0328659102016822,"meas_magnitude":0,"meas_phase_rad":-2.137055383533708,"f_hz":16,"relative_amplitude":0.1}]}}'})
}


setup();
connect();

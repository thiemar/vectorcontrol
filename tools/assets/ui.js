/*
Copyright (c) 2014 - 2015 by Thiemar Pty Ltd

This file is part of vectorcontrol.

vectorcontrol is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

vectorcontrol is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
vectorcontrol. If not, see <http://www.gnu.org/licenses/>.
*/

/* Bah */
NodeList.prototype.forEach = Array.prototype.forEach;
HTMLCollection.prototype.forEach = Array.prototype.forEach;

var ws, deviceCurrentData = {}, deviceSpeedData = {}, deviceCurrentCharts = {},
    deviceSpeedCharts = {}, deviceVoltageTempCharts = {}, lastUpdate = null;

function connect() {
    ws = new WebSocket("ws://" + window.location.host + "/can");
    lastUpdate = (new Date()).valueOf();

    /*
    Handle new messages from the server. These generally map to messages
    received over the CAN interface, although the Python backend aggregates
    some of the high-frequency messages to reduce load.
    */
    ws.onmessage = function(event) {
        var message = JSON.parse(event.data), nodeUi, content, temp;

        if (message.node_id === undefined) {
            return;
        }

        lastUpdate = (new Date()).valueOf();

        /*
        Check for the existence of an interface for this node. If not found,
        clone the template interface.
        */
        nodeUi = document.getElementById("device-" + message.node_id);
        if (!nodeUi) {
            content = document.getElementById("content");
            nodeUi = document.getElementById("device-template").cloneNode(true);
            nodeUi.id = "device-" + message.node_id;
            nodeUi.querySelector("span.node-id").textContent = message.node_id;
            content.appendChild(nodeUi);
            setupSpeedChart(nodeUi);
            setupCurrentChart(nodeUi);
            setupVoltageTempChart(nodeUi);
        }

        /* Process message data */
        if (message.uptime) {
            /* Node status (UAVCAN) -- display the new uptime */
            nodeUi.querySelector("span.node-uptime").textContent = message.uptime;
        } else if (message.param_name) {
            /*
            Parameter value update -- set the corresponding input to the new
            value, and update the chart scaling if necessary.
            */
            if (message.param_name == "pwm_ctl_mode") {
                nodeUi.querySelector("select[name=" + message.param_name + "]"
                    ).selectedIndex = message.value;
            } else {
                nodeUi.querySelector("input[name=" + message.param_name + "]"
                    ).value = parseFloat(message.value).toPrecision(4);
            }

            updateCurrentChart(nodeUi, deviceCurrentData[message.node_id] || []);
            updateSpeedChart(nodeUi, deviceSpeedData[message.node_id] || []);
        } else if (message.current) {
            /*
            Current measurement data -- add it to the measurement array,
            removing old data if the total length is more than 1500 samples
            (30 seconds * 50 Hz). Update the current chart once done.
            */
            if (!deviceCurrentData[message.node_id]) {
                deviceCurrentData[message.node_id] = [];
            }
            Array.prototype.push.apply(deviceCurrentData[message.node_id],
                                       message.current);
            if (deviceCurrentData[message.node_id].length > 1500) {
                deviceCurrentData[message.node_id] =
                    deviceCurrentData[message.node_id].slice(
                        deviceCurrentData[message.node_id].length - 1500);
            }
            updateCurrentChart(nodeUi, deviceCurrentData[message.node_id]);
        } else if (message.speed) {
            /* Speed measurement data -- same deal as for current data */
            if (!deviceSpeedData[message.node_id]) {
                deviceSpeedData[message.node_id] = [];
            }
            Array.prototype.push.apply(deviceSpeedData[message.node_id],
                                       message.speed);
            if (deviceSpeedData[message.node_id].length > 1500) {
                deviceSpeedData[message.node_id] =
                    deviceSpeedData[message.node_id].slice(
                        deviceSpeedData[message.node_id].length - 1500);
            }
            updateSpeedChart(nodeUi, deviceSpeedData[message.node_id]);
            updateVoltageTempChart(nodeUi, deviceSpeedData[message.node_id]);
        }
    }

    /* Send an initial message to trigger the initial fetch */
    ws.onopen = function(event) {
        ws.send("{}");
    }

    /*
    Automatic reconnect on timeout -- check every second and if no messages
    have been received for the last two seconds, close the socket and try
    again.

    Also send an empty message to the server to keep that end alive.
    */
    ws_timeout = window.setInterval(function() {
        if ((new Date()).valueOf() - lastUpdate > 2000.0) {
            window.clearInterval(ws_timeout);
            ws.close();
            connect();
        } else {
            ws.send("{}");
        }
    }, 1000.0);
}


function setupVoltageTempChart(device) {
    var result = { chart: null, x: null, y: null, xAxis: null, yAxis: null },
        svg = device.querySelector("svg.voltagetemp-chart"),
        container = device.querySelector("div.device-measurements"),
        width = container.clientWidth - 150,
        height = 200,
        margin = {top: 10, right: 50, left: 50, bottom: 50};

    result.x = d3.scale.linear()
        .range([0, width])
        .domain([0.0, 30.0]);

    result.y0 = d3.scale.linear()
        .range([height, 0.0])
        .domain([0.0, 27.0]);

    result.y1 = d3.scale.linear()
        .range([height, 0.0])
        .domain([-40.0, 85.0]);

    result.xAxis = d3.svg.axis()
        .scale(result.x)
        .orient("bottom")
        .ticks(30)
        .tickSize(-height, 0, 0);
    result.yAxis0 = d3.svg.axis()
        .scale(result.y0)
        .orient("left");
    result.yAxis1 = d3.svg.axis()
        .scale(result.y1)
        .orient("right");

    result.chart = d3.select(svg)
        .attr("width", width + margin.left + margin.right)
        .attr("height", height + margin.top + margin.bottom)
        .append("g")
            .attr("transform", "translate(" + margin.left + "," + margin.top + ")");

    result.chart.append("g")
        .attr("class", "x axis")
        .attr("transform", "translate(0, " + height + ")")
        .call(result.xAxis)
    .append("text")
        .attr("x", width / 2)
        .attr("y", 30)
        .style("text-anchor", "middle")
        .text("Time (s)");

    result.chart.append("g")
        .attr("class", "y0 axis")
        .call(result.yAxis0)
    .append("text")
        .attr("x", -height / 2)
        .attr("y", -40)
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .text("Bus voltage (V)");

    result.chart.append("g")
        .attr("class", "y1 axis")
        .attr("transform", "translate(" + width + ", 0)")
        .call(result.yAxis1)
    .append("text")
        .attr("x", -height / 2)
        .attr("y", 40)
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .text("Temperature (°C)");

    result.chart.append("clipPath")
        .attr("id", "clip")
    .append("rect")
        .attr("width", width)
        .attr("height", height);

    result.chart.append("path")
        .attr("class", "vbus")
        .attr('clip-path', 'url(#clip)');

    result.chart.append("path")
        .attr("class", "temperature")
        .attr('clip-path', 'url(#clip)');

    deviceVoltageTempCharts[parseInt(device.id.split("-")[1], 10)] = result;
}


function setupCurrentChart(device) {
    var result = { chart: null, x: null, y: null, xAxis: null, yAxis: null },
        svg = device.querySelector("svg.current-chart"),
        container = device.querySelector("div.device-measurements"),
        width = container.clientWidth - 150,
        height = 200,
        margin = {top: 10, right: 50, left: 50, bottom: 10};

    result.x = d3.scale.linear()
        .range([0, width])
        .domain([0.0, 30.0]);

    result.y = d3.scale.linear()
        .range([height, 0.0])
        .domain([0.0, 40.0]);

    result.xAxis = d3.svg.axis()
        .scale(result.x)
        .orient("bottom")
        .ticks(30)
        .tickFormat("")
        .tickSize(-height, 0, 0);
    result.yAxis = d3.svg.axis()
        .scale(result.y)
        .orient("left")
        .tickSize(-width, 0, 0);

    result.chart = d3.select(svg)
        .attr("width", width + margin.left + margin.right)
        .attr("height", height + margin.top + margin.bottom)
        .append("g")
            .attr("transform", "translate(" + margin.left + "," + margin.top + ")");

    result.chart.append("g")
        .attr("class", "x axis")
        .attr("transform", "translate(0, " + height + ")")
        .call(result.xAxis);

    result.chart.append("g")
        .attr("class", "y axis")
        .call(result.yAxis)
    .append("text")
        .attr("x", -height / 2)
        .attr("y", -40)
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .text("Current draw (A)");

    result.chart.append("clipPath")
        .attr("id", "clip")
    .append("rect")
        .attr("width", width)
        .attr("height", height);

    result.chart.append("path")
        .attr("class", "current-setpoint")
        .attr('clip-path', 'url(#clip)');

    result.chart.append("path")
        .attr("class", "current-id")
        .attr('clip-path', 'url(#clip)');

    result.chart.append("path")
        .attr("class", "current-iq")
        .attr('clip-path', 'url(#clip)');

    deviceCurrentCharts[parseInt(device.id.split("-")[1], 10)] = result;
}

function setupSpeedChart(device) {
    var result = { chart: null, x: null, y: null, xAxis: null, yAxis: null },
        svg = device.querySelector("svg.speed-chart"),
        container = device.querySelector("div.device-measurements"),
        width = container.clientWidth - 150,
        height = 200,
        margin = {top: 10, right: 50, left: 50, bottom: 10};

    result.x = d3.scale.linear()
        .range([0, width])
        .domain([0.0, 30.0]);

    result.y = d3.scale.linear()
        .range([height, 0.0])
        .domain([0.0, 10000.0]);

    result.xAxis = d3.svg.axis()
        .scale(result.x)
        .orient("bottom")
        .ticks(30)
        .tickFormat("")
        .tickSize(-height, 0, 0);
    result.yAxis = d3.svg.axis()
        .scale(result.y)
        .orient("left")
        .tickSize(-width, 0, 0);

    result.chart = d3.select(svg)
        .attr("width", width + margin.left + margin.right)
        .attr("height", height + margin.top + margin.bottom)
        .append("g")
            .attr("transform", "translate(" + margin.left + "," + margin.top + ")");

    result.chart.append("g")
        .attr("class", "x axis")
        .attr("transform", "translate(0, " + height + ")")
        .call(result.xAxis);

    result.chart.append("g")
        .attr("class", "y axis")
        .call(result.yAxis)
    .append("text")
        .attr("x", -height / 2)
        .attr("y", -40)
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .text("Speed (rpm)");

    result.chart.append("clipPath")
        .attr("id", "clip")
    .append("rect")
        .attr("width", width)
        .attr("height", height);

    result.chart.append("path")
        .attr("class", "speed-setpoint")
        .attr('clip-path', 'url(#clip)');

    result.chart.append("path")
        .attr("class", "speed-actual")
        .attr('clip-path', 'url(#clip)');

    deviceSpeedCharts[parseInt(device.id.split("-")[1], 10)] = result;
}

function updateVoltageTempChart(device, data) {
    var seriesData, deviceId, chart;

    deviceId = parseInt(device.id.split("-")[1], 10);
    chart = deviceVoltageTempCharts[deviceId];

    /* Vbus line */
    seriesData = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 50.0); })
        .y(function(d) { return chart.y0(d.vbus); });

    chart.chart.select(".vbus")
        .datum(data)
        .attr("d", seriesData);

    /* Temperature line */
    seriesData = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 50.0); })
        .y(function(d) { return chart.y1(d.temperature); });

    chart.chart.select(".temperature")
        .datum(data)
        .attr("d", seriesData);
}

function updateCurrentChart(device, data) {
    var current, deviceId, chart;

    deviceId = parseInt(device.id.split("-")[1], 10);
    chart = deviceCurrentCharts[deviceId];

    chart.y.domain([0.0, parseFloat(device.querySelector("input[name=motor_current_limit]").value) + 1.0]);
    chart.yAxis.scale(chart.y);
    chart.chart.select(".y.axis").call(chart.yAxis);

    /* Setpoint line */
    current = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 50.0); })
        .y(function(d) { return chart.y(Math.abs(d.i_setpoint)); });

    chart.chart.select(".current-setpoint")
        .datum(data)
        .attr("d", current);

    /* Id line */
    current = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 50.0); })
        .y(function(d) { return chart.y(Math.abs(d.i_d)); });

    chart.chart.select(".current-id")
        .datum(data)
        .attr("d", current);

    /* Iq line */
    current = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 50.0); })
        .y(function(d) { return chart.y(Math.abs(d.i_q)); });

    chart.chart.select(".current-iq")
        .datum(data)
        .attr("d", current);
}

function updateSpeedChart(device, data) {
    var speed, deviceId, chart;

    deviceId = parseInt(device.id.split("-")[1], 10);
    chart = deviceSpeedCharts[deviceId];

    chart.y.domain([0.0, parseFloat(device.querySelector("input[name=motor_rpm_max]").value) + 500.0]);
    chart.yAxis.scale(chart.y);
    chart.chart.select(".y.axis").call(chart.yAxis);

    /* Setpoint line */
    speed = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 50.0); })
        .y(function(d) { return chart.y(Math.abs(d.rpm_setpoint)); });

    chart.chart.select(".speed-setpoint")
        .datum(data)
        .attr("d", speed);

    /* Id line */
    speed = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 50.0); })
        .y(function(d) { return chart.y(Math.abs(d.rpm)); });

    chart.chart.select(".speed-actual")
        .datum(data)
        .attr("d", speed);
}

function updateSetpointSchedule(device) {
    var tTotal, tLow, tRise, tHigh, tFall, minSetpoint, maxSetpoint, i, f, d,
        p, schedule, minFreq, maxFreq;

    switch (device.querySelector("select[name=command]").value) {
        case "constant":
            schedule = [parseFloat(device.querySelector("input[name=constant_setpoint]").value) || 0.0]
            break;
        case "step":
            tLow = parseFloat(device.querySelector("input[name=step_low_time]").value || 0.01);
            tRise = parseFloat(device.querySelector("input[name=step_rise_time]").value || 0.01);
            tHigh = parseFloat(device.querySelector("input[name=step_high_time]").value || 0.01);
            tFall = parseFloat(device.querySelector("input[name=step_fall_time]").value || 0.01);
            minSetpoint = parseFloat(device.querySelector("input[name=step_setpoint_start]").value || 0.0);
            maxSetpoint = parseFloat(device.querySelector("input[name=step_setpoint_end]").value || 0.0);
            tTotal = tLow + tRise + tHigh + tFall;

            schedule = [];
            for (i = 0; i < tTotal; i += 0.01) {
                if (i < tLow) {
                    schedule.push(minSetpoint);
                } else if (i >= tLow && i < tLow + tRise) {
                    schedule.push(minSetpoint + (maxSetpoint - minSetpoint) *
                                                (i - tLow) / tRise);
                } else if (i >= tLow + tRise && i < tLow + tRise + tHigh) {
                    schedule.push(maxSetpoint);
                } else if (i >= tLow + tRise + tHigh) {
                    schedule.push(maxSetpoint + (minSetpoint - maxSetpoint) *
                                                (i - tLow - tRise - tHigh) /
                                                tFall);
                }
            }
            break;
        case "sine":
            minSetpoint = parseFloat(device.querySelector("input[name=sine_setpoint_start]").value || 0.0);
            maxSetpoint = parseFloat(device.querySelector("input[name=sine_setpoint_end]").value || 0.0);
            minFreq = parseFloat(device.querySelector("input[name=sine_frequency]").value || 1.0);
            tTotal = 1.0 / minFreq;

            schedule = [];
            for (i = 0, p = 0, d = 2.0 * Math.PI * minFreq / 100.0;
                    i < tTotal; i += 0.01) {
                schedule.push(minSetpoint + (maxSetpoint - minSetpoint) *
                                            (0.5 * (1.0 + Math.sin(p))));
                p += d;
            }
            break;
        case "sweep":
            minSetpoint = parseFloat(device.querySelector("input[name=sweep_setpoint_start]").value || 0.0);
            maxSetpoint = parseFloat(device.querySelector("input[name=sweep_setpoint_end]").value || 0.0);
            minFreq = parseFloat(device.querySelector("input[name=sweep_frequency_start]").value || 1.0);
            maxFreq = parseFloat(device.querySelector("input[name=sweep_frequency_end]").value || 1.0);
            tTotal = parseFloat(device.querySelector("input[name=sweep_rise_time]").value || 1.0);

            schedule = [];
            for (i = 0, p = 0, f = minFreq, d = 2.0 * Math.PI * f / 100.0;
                    i < tTotal * 0.5; i += 0.01) {
                schedule.push(minSetpoint + (maxSetpoint - minSetpoint) *
                                            (0.5 * (1.0 + Math.sin(p))));
                p += d;
                f += (maxFreq - minFreq) / (100.0 * tTotal * 0.5);
                d = 2.0 * Math.PI * f / 100.0;
            }
            for (f = maxFreq, d = 2.0 * Math.PI * f / 100.0;
                    i < tTotal; i += 0.01) {
                schedule.push(minSetpoint + (maxSetpoint - minSetpoint) *
                                            (0.5 * (1.0 + Math.sin(p))));
                p += d;
                f += (minFreq - maxFreq) / (100.0 * tTotal * 0.5);
                d = 2.0 * Math.PI * f / 100.0;
            }

            break;
    }

    ws.send(JSON.stringify({
        node_id: parseInt(device.id.split("-")[1], 10),
        schedule: schedule.length ? schedule : [0]
    }));
}

function setupEventListeners() {
    var content, nodeUi;

    content = document.getElementById("content");
    content.addEventListener("change", function(event) {
        if (event.target.classList.contains("configuration")) {
            nodeUi = event.target.parentElement.parentElement.parentElement
                                 .parentElement.parentElement;
            ws.send(JSON.stringify({
                param_name: event.target.name,
                param_value: parseFloat(event.target.name !== "pwm_ctl_mode" ?
                                            event.target.value :
                                            event.target.selectedIndex),
                node_id: parseInt(nodeUi.id.split("-")[1], 10)
            }));
        } else if (event.target.name == "command") {
            nodeUi = event.target.parentElement.parentElement.parentElement
                                 .parentElement.parentElement;
            event.target.parentNode.parentNode.querySelectorAll("fieldset")
                                              .forEach(function(elem) {
                if (elem.classList.contains("command-" + event.target.value)) {
                    elem.classList.remove("hidden");
                } else {
                    elem.classList.add("hidden");
                }
            });
            updateSetpointSchedule(nodeUi);
        } else if (event.target.name == "mode") {
            nodeUi = event.target.parentElement.parentElement.parentElement
                                 .parentElement.parentElement;
            ws.send(JSON.stringify({
                mode: event.target.value == "torque" ? 3 : 2,
                node_id: parseInt(nodeUi.id.split("-")[1], 10)
            }));
        } else if (event.target.classList.contains("setpoint")) {
            nodeUi = event.target.parentElement.parentElement.parentElement
                                 .parentElement.parentElement.parentElement;
            updateSetpointSchedule(nodeUi);
        }
    });

    content.addEventListener("click", function(event) {
        if (event.target.classList.contains("apply-configuration")) {
            nodeUi = event.target.parentElement.parentElement.parentElement
                                 .parentElement;
            ws.send(JSON.stringify({
                param_apply: true,
                node_id: parseInt(nodeUi.id.split("-")[1], 10)
            }));
        } else if (event.target.classList.contains("motor-start")) {
            nodeUi = event.target.parentElement.parentElement.parentElement
                                 .parentElement;
            nodeUi.querySelectorAll("input, select").forEach(function(elem) {
                if (elem.name == "stop") {
                    elem.disabled = false;
                } else {
                    elem.disabled = true;
                }
            });

            ws.send(JSON.stringify({
                motor_running: true,
                node_id: parseInt(nodeUi.id.split("-")[1], 10)
            }));
        } else if (event.target.classList.contains("motor-stop")) {
            nodeUi = event.target.parentElement.parentElement.parentElement
                                 .parentElement;
            nodeUi.querySelectorAll("input, select").forEach(function(elem) {
                if (elem.name == "stop") {
                    elem.disabled = true;
                } else {
                    elem.disabled = false;
                }
            });

            ws.send(JSON.stringify({
                motor_running: false,
                node_id: parseInt(nodeUi.id.split("-")[1], 10)
            }));
        }
    });
}

connect();
setupEventListeners();


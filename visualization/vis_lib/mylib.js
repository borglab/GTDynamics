
var width = 1800,
    height = 900,
    margin = 20,
    radius = 20,
    rect_w = 20,
    rect_h = 20;

var graph_x = 200,
    graph_w = 1000,
    graph_y = 100,
    graph_h = 600;

var plot_width = 400,
    plot_height = 300,
    margin_x = 50,
    margin_y = 50,
    plot_x = width - margin_x - plot_width,
    plot_y = height - margin_y - plot_height;



/**
 * @brief   write text and figure to the canvas
 */
function write_text(svg) 
{
    // svg.append("rect")
    //     .attr("width", "100%").attr("height", "100%")
    //             .attr("fill", "#eeeeee");

    svg.append("text")
        .attr("x", margin)
        .attr("y", margin)
        .attr("text-anchor", "front")
        .attr("font-family", "sans-serif")
        .attr("font-size", "20px")
        .attr("fill", "green")
        .text("Factor Graph Visualization");

    svg.append("text")
        .attr("x", margin)
        .attr("y", margin+20)
        .attr("text-anchor", "front")
        .attr("font-family", "sans-serif")
        .attr("font-size", "16px")
        .attr("fill", "green")
        .text("- double click to fix/unfix the nodes");

    svg.append("text")
        .attr("x", margin)
        .attr("y", margin+40)
        .attr("text-anchor", "front")
        .attr("font-family", "sans-serif")
        .attr("font-size", "16px")
        .attr("fill", "green")
        .text("- click to show detailed info");

    svg.append("svg:image")
        .attr('x', 200)
        .attr('y', height-100)
        .attr('width', 600)
        .attr('height', 30)
        .attr("xlink:href", "https://raw.githubusercontent.com/d3/d3-scale-chromatic/master/img/plasma.png")

    svg.append("text")
        .attr("x", 400)
        .attr("y", height-50)
        .attr("text-anchor", "middle")
        .attr("font-family", "sans-serif")
        .attr("font-size", "15px")
        .attr("fill", "darkblue")
        .text("less error");

    svg.append("text")
        .attr("x", 600)
        .attr("y", height-50)
        .attr("text-anchor", "middle")
        .attr("font-family", "sans-serif")
        .attr("font-size", "15px")
        .attr("fill", "darkblue")
        .text("more error");
}


/**
 * @brief       sort factors by errors, to display the factor with larger error on top
 * @param[in]   factors
 * @return      sorted factors
 */
function sort_factors_by_error(factors)
{
    factors.sort(function(a,b) {
        return Number(a.error)-Number(b.error);
    });
    return factors;
}

/**
 * @brief 		get nodes and links from variables and factors
 * @param[in] 	variables
 * @param[in] 	factors
 * @return		nodes_dict and links
 */
function get_nodes_links(variables, factors) 
{
    var nodes_dict = {};
    factors.forEach(function (factor) {
        factor.node_type = "factor";
        factor.links = new Set([]);
        nodes_dict[factor.name] = factor;
    });

    variables.forEach(function (variable) {
        variable.node_type = "variable";
        variable.links = new Set([]);
        nodes_dict[variable.name] = variable;
    });

    var links = [];
    factors.forEach(function (factor) {
        factor["variables"].forEach(function (variable_name) {
            link = {source: factor, target: nodes_dict[variable_name], selected:false};
            links.push(link);
            factor.links.add(link);
            nodes_dict[variable_name].links.add(link);
        })
    });

    return [nodes_dict, links];
}

/**
 * @brief 		set error-color scale for factor nodes
 * @param[in] 	factors
 * @return		a scale mapping from error value to color
 */
function get_error_scale(factors)
{
    var min_error = facotrs[0].error,
        max_error = factors[0].error;
    if (factors.length > 0)
    {
        max_error = factors[factors.length - 2].error; // ignore the largest error, since it is commonly from the prior factor
    }

    return d3.scaleSequential(d3.interpolatePlasma)
        	.domain([min_error, max_error]);
}

/**
 * @brief 		fix the nodes with specified locations
 * @param[in] 	nodes_dict
 */
function fix_nodes(nodes_dict)
{
    locations = [];
    for (key in nodes_dict) {
        if (nodes_dict[key].hasOwnProperty("location")) {
            locations.push(nodes_dict[key].location)
        }
        nodes_dict[key].selected = false;
        nodes_dict[key].value_idx = 0;
    }

    if (locations.length>1) {
        var max_x = Math.max.apply(Math, locations.map(function (f) {
            return f[0];
        }));
        var min_x = Math.min.apply(Math, locations.map(function (f) {
            return f[0];
        }));
        var max_y = Math.max.apply(Math, locations.map(function (f) {
            return f[1];
        }));
        var min_y = Math.min.apply(Math, locations.map(function (f) {
            return f[1];
        }));
        var xScale = d3.scaleLinear().domain([min_x, max_x]).range([graph_x, graph_x+graph_w]);
        var yScale = d3.scaleLinear().domain([min_y, max_y]).range([graph_y, graph_y+graph_h]);

        for (key in nodes_dict) {
            if (nodes_dict[key].hasOwnProperty("location")) {
                nodes_dict[key].fx = xScale(nodes_dict[key].location[0]);
                nodes_dict[key].fy = yScale(nodes_dict[key].location[1]);
                nodes_dict[key].fixed = true;
            }
            else {
                nodes_dict[key].fixed = false;
            }
        }
    }
}

/**
 * @brief 		display an attribute as row in table of tool tip
 * @param[in] 	key: atrribute name
 * @param[in]	value: attribute value
 */
function table_entry(key, value) {
    return "<tr><th>" + key + "</th> <td>" + value + "</td></tr>";
}

/**
 * @brief 		get the tip function
 */
function get_tip()
{
    tip = d3.tip()
        .attr("fill", "white")
        .attr('class', 'd3-tip').html(function (d) {
            var node_name = d.name
            if (d.node_type == "variable") {
                var text = "<table>"
                text += table_entry("name", d.name);
                if ("value" in d) {
                    text += table_entry("value", d.value);
                }
                text+="</table>";
                return text;
            }
            if (d.node_type == "factor") {
                var text = "<table>"
                text += table_entry("name", d.name);
                if ("type" in d) {
                    text += table_entry("type", d.type);
                }
                if ("variables" in d) {
                    text += table_entry("variables", d.variables)
                }
                if ("measurement" in d) {
                    text += table_entry("measurement", d.measurement)
                }
                if ("noise" in d) {
                    text +=  table_entry("noise", d.noise);
                }
                if ("error" in d) {
                    text += table_entry("error", "<u>" + d.error+ "</u>");
                }
                text+="</table>";
                return text;
            }
            return "unrecognized node";
        });
    return tip;
}

/**
 * @brief 		create force simulation
 * @param[in] 	nodes_dict
 * @param[in]	links
 * @return		[force, path, node]
 */
function force_simulation(nodes_dict, links)
{
    var force = d3.forceSimulation()
        .nodes(d3.values(nodes_dict))
        .force("link", d3.forceLink(links).distance(50))
        .force("collide", d3.forceCollide(20))
        // .force('center', d3.forceCenter(width / 2, height / 2))
        // .force("x", d3.forceX())
        // .force("y", d3.forceY())
        // .force("charge", d3.forceManyBody().strength(0)) // todo
        .alphaTarget(1)
        .on("tick", tick);

    var path = svg.append("g")
        .selectAll("path")
        .data(links)
        .enter()
        .append("path")
        .attr("class", "link");

    path.style('stroke', "#deddff")
        .style('stroke-width', '2.0px')
        .attr("id", function(d) {
            return "p"+d.index;
        });

    var node = svg.selectAll(".node")
        .data(force.nodes())
        .enter().append("g")
        .attr("class", "node");

    function tick() {
        path.attr("d", function (d) {
            var dx = d.target.x - d.source.x,
                dy = d.target.y - d.source.y,
                // dr = Math.sqrt(dx * dx + dy * dy);
                dr = 0;
            return "M" +
                d.source.x + "," +
                d.source.y + "A" +
                dr + "," + dr + " 0 0,1 " +
                d.target.x + "," +
                d.target.y;
        });

        node.attr("transform", function (d) {
            var radius = 20 // todo
            var x = d.x
            var y = d.y
            x = Math.max(radius, Math.min(width - radius, d.x));
            y = Math.max(radius, Math.min(height - radius, d.y));
            return "translate(" + x + "," + y + ")";
        })
    }

    return [force, path, node];
}

/**
 * @brief 		split nodes into variables nodes and factor nodes
 * @param[in] 	nodes
 * @return		[variable_nodes, factor_nodes]
 */
function split_nodes(nodes) {
    var variable_nodes = nodes.filter(function(d, i) {
            return d.node_type == "variable"
        });

    var factor_nodes = nodes.filter(function(d, i) {
            return d.node_type == "factor"
        });
   	return [variable_nodes, factor_nodes];
}

/**
 * @brief 		draw the nodes
 * @param[in] 	variable_nodes
 * @param[in]	factor_nodes
 * @param[in]	errorScale
 */
function draw_nodes(variable_nodes, factor_nodes, errorScale) 
{

    var factor_shapes = factor_nodes.append("rect")
        .attr("id", function(d) {return d.name;})
        .attr("x", -rect_w/2)
        .attr("y", -rect_h/2)
        .attr('width', rect_w)
        .attr('height', rect_h)
        .style('fill', function (d) {return errorScale(d.error)})
        .style("stroke", function (d) {return d.fixed? "black" :null})
        .style("stroke-width", function (d) {return d.fixed? "2.5px" :null});;

    var variable_shapes = variable_nodes.append("circle")
        .attr("id", function(d) {return d.name;})
        .attr("r", radius)
        .style('fill', function (d) {return "#cccccc"})
        .style("stroke", function (d) {return d.fixed? "black" :null})
        .style("stroke-width", function (d) {return d.fixed? "2.5px" :null});

    var variable_text = variable_nodes.append("text")
        .attr("text-anchor", "middle")
        .attr('dominant-baseline', 'middle')
        .text(function(d) {return d.name});

    // var factor_text = factor_nodes.append("text")
    //     .attr("y", rect_h/2)
    //     .attr("text-anchor", "middle")
    //     .attr('dominant-baseline', 'hanging')
    //     .text(function(d) {return d.type});
}


/**
 * @brief 		set the node to be unselected
 * @param[in] 	d_node
 */
function set_unselected(d_node)
{
	let node_circle = d3.select('#'+ d_node.name);
    d_node.selected = false;
    node_circle.style("stroke", d_node.fixed? "black" :null).style("stroke-width", d_node.fixed? "2.5px" :null);
}

/**
 * @brief 		set the node to be selected
 * @param[in] 	d_node
 */
function set_selected(d_node)
{
	let node_circle = d3.select('#'+ d_node.name);
    d_node.selected = true;
    node_circle.style("stroke", "red");
}

/**
 * @brief 		clear line chart
 * @param[in] 	line_chart
 */
 function clear_line_chart(line_chart)
 {
    var data_empty = [];
    line_chart.selectAll(".line").data(data_empty).exit().remove();
    line_chart.selectAll(".axis").remove();
    line_chart.selectAll(".label").remove();
 }

/**
 * @brief 		draw line chart
 * @param[in] 	line_chart
 * @param[in] 	d_node: the selected node
 */
function draw_line_chart(line_chart, d_node)
{
    var value_type = d3.select('select').property('value');
    var value_idx = d_node.value_types.indexOf(value_type);

    var value_history = d_node.value_history;
	var time_steps = value_history.length;
    var data = []
    for (var i=0; i<time_steps; i++) {
        data.push({"index": i, "data": value_history[i][value_idx]})
    }

    var max_value = d3.max(data, function(d) {return d["data"];});
    var min_value = d3.min(data, function(d) {return d["data"];});
    var line_chart_xScale = d3.scaleLinear().domain([0, time_steps-1]).range([plot_x, plot_x + plot_width]);
    var line_chart_yScale = d3.scaleLinear().domain([min_value, max_value])
                                            .range([plot_y + plot_height, plot_y]);

    var line_function = d3.line()
        .x(function(d) {return line_chart_xScale(d["index"]); })
        .y(function(d) {return line_chart_yScale(d["data"]); })

    line_chart.append("path")
            .datum(data)
            .attr("class", "line")
            .attr("d", line_function)
            .attr("stroke", "RoyalBlue")
            .attr("stroke-width", 2)
            .attr("fill", "transparent");

    // axes
    var xAxis = d3.axisBottom()
          .scale(line_chart_xScale)
          .ticks(time_steps-1);

    var yAxis = d3.axisLeft()
          .scale(line_chart_yScale)
          .ticks(8);

    line_chart.append("g")
        .attr("class", "axis")
        .attr("transform", "translate("+0+"," + (plot_y + plot_height) + ")")
        .call(xAxis);

    line_chart.append("g")
        .attr("class", "axis")
        .attr("transform", "translate("+(plot_x)+","+0+")")
        .call(yAxis);

    // x label
    line_chart.append('text')
        .attr('class', 'label')
        .attr('x', plot_x + plot_width)
        .attr('y', plot_y + plot_height+25)
        .attr('text-anchor', 'middle')
        .attr('dominant-baseline', 'baseline')
        .attr("font-size", "14px")
        .text('TimeStep');

    // y label
    line_chart.append('text')
        .attr('class', 'label')
        .attr('x', plot_x)
        .attr('y', plot_y-20)
        .attr('text-anchor', 'end')
        .attr('dominant-baseline', 'hanging')
        .attr("font-size", "14px")
        .text(d_node.value_types[value_idx]);
}


/**
 * @brief 		set the option selection bar
 * @param[in] 	d_node: the selected node
 */
function set_options(d_node)
{
    var body = d3.select("body")

    var select = d3.select('.chartArea')
        .append('select')
        .attr('class','select')
        .style('top', plot_y -60 + 'px')
        .style('left', plot_x+'px')

    var options = select
        .selectAll('option')
        .data(d_node.value_types).enter()
        .append('option')
        .attr("class", "selection")
        .text(function (d) { return d; });
    return select;
}

/**
 * @brief 		clear the option selection bar
 */
function clear_options()
{
    // d3.selectAll('option').remove();
    d3.selectAll("#line_chart_svg").remove();
    d3.selectAll(".select").remove();

}
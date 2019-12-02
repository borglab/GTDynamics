// =================== canvas layout =================== //
var chart_div = d3.select("body").append('div').attr('class', 'chartArea');

var svg = chart_div.append("svg")
            .attr("width", width)
            .attr("height", height);

write_text(svg);

var line_chart = svg.append("g")
                    .attr('class', 'line_chart');

// =================== load data =================== //
var file = "factor_graph.json";
Promise.all([d3.json(file)])
        .then(function(data) 
        {
            draw_factor_graph(data[0])
        });


/**
 * @brief       draw factor graph with provided data
 * @param[in]   data: [variables, factors]
 */
function draw_factor_graph(data) {

    // =================== load data =================== //
    variables = data[0];
    factors = data[1];
    facotrs = sort_factors_by_error(factors);
    var [nodes_dict, links] = get_nodes_links(variables, factors);

    console.log("nodes_dict", nodes_dict);
    console.log("links", links);

    // =================== create color and scale mapping =================== //
    var errorScale = get_error_scale(factors);

    // =================== arrange locations of fixed nodes =================== //
    fix_nodes(nodes_dict);

    // =================== tool tip =================== //
    tip = get_tip();
    svg.call(tip);

    // =================== create force simulation =================== //
    var [force, path, nodes] = force_simulation(nodes_dict, links);
    console.log("force", force);

    // =================== draw nodes =================== //
    var [variable_nodes, factor_nodes] = split_nodes(nodes);
    draw_nodes(variable_nodes, factor_nodes, errorScale);

    // =================== nodes interaction options =================== //
    nodes.on('mouseover', mouse_over)
        .on('mouseout', mouse_out)
        .call(d3.drag()
            .on("start", dragstarted)
            .on("drag", dragged)
            .on("end", dragended)
        );

    factor_nodes.on("dblclick", pin);

    var timeout = null;
    variable_nodes//.on("click", make_plot);
    .on("click", function(d) {
        clearTimeout(timeout);
        timeout = setTimeout(function() {
          make_plot(d);
        }, 200)
      })
      .on("dblclick", function(d) {
        clearTimeout(timeout);
        pin(d);
      });

    // =================== mouse over =================== //

    function mouse_over(d, i) {
        tip.show(d, this)
        if (d.node_type=="variable")
        {
            for (let l1 of d.links) {
                var factor_node = l1.source; //source is always factor
                for (let l2 of factor_node.links) {
                    let p = d3.select('#p' + l2.index)
                    p.style('stroke', "gold")
                }
            }
        }
        else
        {
            for (let l of d.links) {
                let p = d3.select('#p' + l.index)
                p.style('stroke', "gold")
            }
        }


    }

    function mouse_out(d, i) {
        tip.hide(d, this)
        if (d.node_type=="variable")
        {
            for (let l1 of d.links) {
                var factor_node = l1.source; //source is always factor
                for (let l2 of factor_node.links) {
                    let p = d3.select('#p' + l2.index)
                    p.style('stroke', "#deddff")
                }
            }
        }
        else
        {
            for (let l of d.links) {
                let p = d3.select('#p' + l.index)
                p.style('stroke', "#deddff")
            }
        }

    }

    // =================== drag =================== //
    function dragstarted(d) {
        if (!d3.event.active) force.alphaTarget(0.3).restart();
        d.fx = d.x;
        d.fy = d.y;
    }

    function dragged(d) {
        d.fx = d3.event.x;
        d.fy = d3.event.y;
    }

    function dragended(d) {
        if (!d3.event.active) force.alphaTarget(0);
        if (d.fixed) {
            d.fx = d.x;
            d.fy = d.y;
        } else {
            d.fx = null;
            d.fy = null;
        }

    }

    // =================== click =================== //
    function pin(d) {
        d.fixed = !d.fixed;
        dragstarted(d);
        dragged(d);
        dragended(d);
        let c = d3.select('#'+ d.name)
        c.style("stroke", d.fixed? "black" :null).style("stroke-width", d.fixed? "2.5px" :null)      
    };

    // =================== double click =================== //
    function make_plot(d_node) {
        // remove the old plot
        clear_line_chart(line_chart);
        clear_options();

        // if clicking on the same node, just unselect it
        if (d_node.selected) {
            set_unselected(d_node);
            return;
        }

        // unselect the previous node, and update previous node
        if (typeof make_plot.prev_node != 'undefined') {
            set_unselected(make_plot.prev_node);
        }
        make_plot.prev_node = d_node;

        // set the new node to be selected
        set_selected(d_node);

        // set the options
        select = set_options(d_node);
        select.on('change',onchange);
        onchange();

        function onchange() {
            clear_line_chart(line_chart);
            draw_line_chart(line_chart, d_node);
        }
    }
} // draw_factor_graph
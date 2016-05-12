http_init
gr.name = 'grupo1';
gr.resources{1} = '/motion/vel';
gr.resources{2} = '/motion/pose';
gr.resources{3} = '/perception/laser/1/distances?range=-30:30:2';
g1 = http_post('http://127.0.0.1:4950/group', gr);

data = http_get(g1);

data

http_delete(g1);
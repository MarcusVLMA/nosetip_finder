const nosetip_finder = require('bindings')('nosetip_finder');

let nosetip = nosetip_finder.findNoseTip('./bs000_E_DISGUST_0_Fall2003.pcd');
console.log('NOSETIP',nosetip)
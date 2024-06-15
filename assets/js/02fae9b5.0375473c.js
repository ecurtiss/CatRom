"use strict";(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[331],{3905:(e,t,n)=>{n.d(t,{Zo:()=>c,kt:()=>f});var a=n(67294);function o(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function r(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);t&&(a=a.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,a)}return n}function i(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?r(Object(n),!0).forEach((function(t){o(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):r(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function l(e,t){if(null==e)return{};var n,a,o=function(e,t){if(null==e)return{};var n,a,o={},r=Object.keys(e);for(a=0;a<r.length;a++)n=r[a],t.indexOf(n)>=0||(o[n]=e[n]);return o}(e,t);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);for(a=0;a<r.length;a++)n=r[a],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(o[n]=e[n])}return o}var p=a.createContext({}),s=function(e){var t=a.useContext(p),n=t;return e&&(n="function"==typeof e?e(t):i(i({},t),e)),n},c=function(e){var t=s(e.components);return a.createElement(p.Provider,{value:t},e.children)},u="mdxType",m={inlineCode:"code",wrapper:function(e){var t=e.children;return a.createElement(a.Fragment,{},t)}},d=a.forwardRef((function(e,t){var n=e.components,o=e.mdxType,r=e.originalType,p=e.parentName,c=l(e,["components","mdxType","originalType","parentName"]),u=s(n),d=o,f=u["".concat(p,".").concat(d)]||u[d]||m[d]||r;return n?a.createElement(f,i(i({ref:t},c),{},{components:n})):a.createElement(f,i({ref:t},c))}));function f(e,t){var n=arguments,o=t&&t.mdxType;if("string"==typeof e||o){var r=n.length,i=new Array(r);i[0]=d;var l={};for(var p in t)hasOwnProperty.call(t,p)&&(l[p]=t[p]);l.originalType=e,l[u]="string"==typeof e?e:o,i[1]=l;for(var s=2;s<r;s++)i[s]=n[s];return a.createElement.apply(null,i)}return a.createElement.apply(null,n)}d.displayName="MDXCreateElement"},76647:(e,t,n)=>{n.r(t),n.d(t,{contentTitle:()=>i,default:()=>u,frontMatter:()=>r,metadata:()=>l,toc:()=>p});var a=n(87462),o=(n(67294),n(3905));const r={},i=void 0,l={type:"mdx",permalink:"/CatRom/",source:"@site/pages/index.md",description:"CatRom",frontMatter:{}},p=[{value:"Introduction",id:"introduction",level:2},{value:"Reparametrization",id:"reparametrization",level:3},{value:"Installation",id:"installation",level:2},{value:"Getting started",id:"getting-started",level:2}],s={toc:p},c="wrapper";function u(e){let{components:t,...n}=e;return(0,o.kt)(c,(0,a.Z)({},s,n,{components:t,mdxType:"MDXLayout"}),(0,o.kt)("p",{align:"center"},(0,o.kt)("h1",null,"CatRom"),"Creates Catmull-Rom splines for Roblox"),(0,o.kt)("h2",{id:"introduction"},"Introduction"),(0,o.kt)("p",null,"A ",(0,o.kt)("a",{parentName:"p",href:"https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline"},"Catmull-Rom spline")," is a C\xb9 cubic spline that\npasses through each of its control points. It is a good choice for your project if you want a smooth curve that"),(0,o.kt)("ol",null,(0,o.kt)("li",{parentName:"ol"},"passes through every control point and"),(0,o.kt)("li",{parentName:"ol"},"does not need any extra data to sculpt it (cf. B\xe9zier curves).")),(0,o.kt)("p",null,"In addition to the control points, CatRom provides two parameters to adjust the spline:"),(0,o.kt)("ol",null,(0,o.kt)("li",{parentName:"ol"},(0,o.kt)("inlineCode",{parentName:"li"},"alpha"),": A number (usually in ","[0, 1]",") that loosely affects the curvature of the spline at the control points\n(default: 0.5)"),(0,o.kt)("li",{parentName:"ol"},(0,o.kt)("inlineCode",{parentName:"li"},"tension"),": A number (usually in ","[0, 1]",") that makes the spline more or less taut (default: 0)")),(0,o.kt)("h3",{id:"reparametrization"},"Reparametrization"),(0,o.kt)("p",null,"By construction, a Catmull-Rom spline clusters points in regions of higher curvature. This effect is often visually\nunappealing, so CatRom offers a second parametrization\u2014called a unit-speed (or arc length) parametrization\u2014that yields\nequally-spaced points given equally-spaced times."),(0,o.kt)("p",null,"By default, passing ",(0,o.kt)("inlineCode",{parentName:"p"},"true")," into a method with a ",(0,o.kt)("inlineCode",{parentName:"p"},"unitSpeed")," argument performs a slow but highly accurate\nreparametrization. If you are calling a high volume of methods with ",(0,o.kt)("inlineCode",{parentName:"p"},"unitSpeed")," true, then you should instead call\n",(0,o.kt)("inlineCode",{parentName:"p"},"CatRom:PrecomputeUnitSpeedData")," beforehand; after the initial cost of the precompute step, this method will make your\nreparametrizations significantly faster at a small cost to accuracy."),(0,o.kt)("h2",{id:"installation"},"Installation"),(0,o.kt)("p",null,"CatRom is available on ",(0,o.kt)("a",{parentName:"p",href:"https://wally.run/"},"Wally")),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-toml"},'catrom = "ecurtiss/catrom@=1.0.0-rc1"\n')),(0,o.kt)("p",null,"or as a ",(0,o.kt)("inlineCode",{parentName:"p"},".rbxm")," from the ",(0,o.kt)("a",{parentName:"p",href:"https://github.com/ecurtiss/CatRom/releases"},"Releases")," page."),(0,o.kt)("h2",{id:"getting-started"},"Getting started"),(0,o.kt)("p",null,"Here is an annotated example to get you started. Thorough documentation can be found\n",(0,o.kt)("a",{parentName:"p",href:"https://ecurtiss.github.io/CatRom/"},"here"),"."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-lua"},"local CatRom = require(path.to.CatRom)\n\nlocal points = {...} -- A list of Vector2s, Vector3s, or CFrames\nlocal alpha = 0.5\nlocal tension = 0\n\nlocal spline = CatRom.new(points, alpha, tension)\n\n-- Get the position at a single time\nlocal pos = spline:SolvePosition(0.5)\n\n-- Get the position at 100 times between 0 and 1 (inclusive)\nspline:SolveBulk(function(segment, t)\n    local pos = segment:SolvePosition(t)\nend, 100, 0, 1)\n\n-- Repeat the above with a unit-speed parametrization\nspline:PrecomputeUnitSpeedData() -- Optional; makes the math faster for bulk computations\nlocal pos = spline:SolvePosition(0.5, true) -- Notice the `true`to indicate unit speed\nspline:SolveBulk(function(segment, t)\n    local pos = segment:SolvePosition(t)\nend, 100, 0, 1, true) -- Notice the `true` to indicate unit speed\n\n-- Smoothly sweep a CFrame from time 0 to 1 with minimal twisting\nlocal initialCF = CFrame.identity\nlocal interpolant = spline:GetTransportInterpolant(initialCF, 0, 1)\nfor i = 0, 100 do\n    local sweptCF = interpolant(i / 100)\nend\n")))}u.isMDXComponent=!0}}]);
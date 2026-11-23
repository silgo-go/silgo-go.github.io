/* 실고 서비스워커: 홈 화면 설치 + 오프라인 실행
   - 페이지(실고 /, 필고 /pilgo): 주소별로 따로 저장, 인터넷 먼저 → 안 되면 저장해 둔 것 (새 버전이 바로 반영되게)
   - 아이콘·글꼴·Firebase 프로그램 파일: 저장해 둔 것 먼저 보여주고 뒤에서 새로 받아 둠
   - 로그인·데이터 통신(Firebase 서버)은 건드리지 않음 */
const V="silgo-v2",CORE=["/","/manifest.webmanifest","/icon-192.png","/icon-512.png","/icon-180.png"];
self.addEventListener("install",e=>{e.waitUntil(caches.open(V).then(c=>c.addAll(CORE)).then(()=>self.skipWaiting()))});
self.addEventListener("activate",e=>{e.waitUntil(caches.keys().then(ks=>Promise.all(ks.filter(k=>k!==V).map(k=>caches.delete(k)))).then(()=>self.clients.claim()))});
self.addEventListener("fetch",e=>{const r=e.request;if(r.method!=="GET")return;const u=new URL(r.url);
 if(r.mode==="navigate"){const key=u.pathname.replace(/\.html$/,"").replace(/\/index$/,"/")||"/";e.respondWith(fetch(r).then(res=>{if(res.ok){const cp=res.clone();caches.open(V).then(c=>c.put(key,cp))}return res}).catch(()=>caches.match(key).then(m=>m||caches.match("/"))));return}
 const mine=u.origin===self.location.origin,lib=/^(fonts\.googleapis\.com|fonts\.gstatic\.com)$/.test(u.hostname)||(u.hostname==="www.gstatic.com"&&u.pathname.startsWith("/firebasejs/"));
 if(!mine&&!lib)return;
 e.respondWith(caches.open(V).then(c=>c.match(r).then(hit=>{const net=fetch(r).then(res=>{if(res&&(res.ok||res.type==="opaque"))c.put(r,res.clone());return res}).catch(()=>hit);return hit||net})))});

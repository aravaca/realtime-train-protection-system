const { createProxyMiddleware } = require('http-proxy-middleware');

module.exports = function(app) {
  app.use(
    '/control',
    createProxyMiddleware({
      target: 'http://localhost:8080',  // 백엔드 서버 실제 주소
      changeOrigin: true,
    })
  );
};

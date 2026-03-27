import log from 'loglevel'

const isDebug =
  new URLSearchParams(location.search).has('debug') ||
  localStorage.getItem('WEB_UI_DEBUG') === 'true'

log.setLevel(isDebug ? 'debug' : 'info')

export default log

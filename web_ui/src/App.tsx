import { useEffect, useMemo, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { LanguageSwitch } from './components/LanguageSwitch';
import { RobotControlContract, TeachStep, webBridgeApi } from './services/webBridgeApi';

const modes = ['control', 'teach', 'visual', 'intelligent'] as const;

const EMPTY_CONTRACT: RobotControlContract = {
  robot_state: 'UNAVAILABLE',
  connection_state: 'UNAVAILABLE',
  armed: false,
  mode: 'Unavailable',
  cycle_ms: 0,
  loss_percent: 0,
  error_message: null,
  can_retry: false,
  request: {
    action: 'none',
    status: 'idle',
    message: null,
  },
  joints: {},
};

export function App() {
  const { t } = useTranslation(['common', 'control']);
  const teachTypes = t('control:cards.teachTypes.items', { returnObjects: true }) as string[];

  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [state, setState] = useState<RobotControlContract>(EMPTY_CONTRACT);
  const [steps, setSteps] = useState<TeachStep[]>([]);
  const activeMode = state.mode.toLowerCase();

  const sortedJoints = useMemo(() => Object.entries(state.joints).sort(([a], [b]) => a.localeCompare(b)), [state.joints]);

  const requestStatusLabel = {
    idle: t('common:requestIdle'),
    in_progress: t('common:requestInProgress'),
    success: t('common:requestSuccess'),
    failed: t('common:requestFailed'),
  }[state.request.status] ?? state.request.status;

  const loadDashboard = async () => {
    setLoading(true);
    setError(null);
    try {
      const [stateRes, seqRes] = await Promise.all([webBridgeApi.getState(), webBridgeApi.getTeachSequence()]);
      setState(stateRes);
      setSteps(seqRes.steps);
    } catch (e) {
      const message = e instanceof Error ? e.message : 'unknown_error';
      setError(message);
      setState({ ...EMPTY_CONTRACT, error_message: message, can_retry: true, request: { action: 'load', status: 'failed', message } });
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    void loadDashboard();
  }, []);

  const toggleArmed = async () => {
    try {
      const resp = state.armed ? await webBridgeApi.disable() : await webBridgeApi.enable();
      setState(resp.state);
      setError(null);
    } catch (e) {
      const message = e instanceof Error ? e.message : 'toggle_failed';
      setError(message);
      setState((prev) => ({
        ...prev,
        error_message: message,
        can_retry: true,
        request: { action: prev.armed ? 'disable' : 'enable', status: 'failed', message },
      }));
    }
  };

  const switchDisabled = loading || state.robot_state === 'FAULT' || state.connection_state !== 'CONNECTED';
  const switchStatusLabel = switchDisabled ? t('common:armedUnavailable') : state.armed ? t('common:armedAuthorized') : t('common:armedStandby');
  const alertTone = !state.can_retry && state.connection_state !== 'CONNECTED' ? 'alert-muted' : state.can_retry ? 'alert-warn' : 'alert-info';

  return (
    <div className="page-bg">
      <div className="ambient ambient-1" />
      <div className="ambient ambient-2" />
      <div className="app-shell">
        <header className="top-bar">
          <div className="brand-block">
            <p>{t('common:controlConsole')}</p>
            <h1>Robot Arm 4338</h1>
          </div>
          <div className="mode-segment" role="tablist" aria-label="mode switch">
            {modes.map((modeItem) => (
              <button key={modeItem} className={modeItem === activeMode ? 'active' : ''}>
                {t(`control:modes.${modeItem}`)}
              </button>
            ))}
          </div>
          <LanguageSwitch />
        </header>

        <main className="layout-grid">
          <aside className="panel telemetry-panel">
            <div className="panel-head"><span>{t('common:telemetry')}</span></div>

            {(error || state.error_message) && (
              <div className={`inline-alert ${alertTone}`}>
                <span>⚠</span>
                <em>{state.error_message ?? error}</em>
                {(state.can_retry || Boolean(error)) && <button onClick={() => void loadDashboard()}>{t('common:retry')}</button>}
              </div>
            )}

            <div className="state-highlight">
              <label>{t('common:robotState')}</label>
              <strong>{loading ? t('common:loading') : state.robot_state}</strong>
              <small>{t('common:requestStatus')}: {requestStatusLabel}</small>
            </div>

            <div className="armed-switch-card">
              <div className="armed-headline">
                <label>{t('common:armed')}</label>
                <span className={`armed-status-chip ${state.armed ? 'armed-chip-on' : 'armed-chip-off'} ${switchDisabled ? 'armed-chip-disabled' : ''}`}>{switchStatusLabel}</span>
              </div>
              <button className={`armed-switch ${state.armed ? 'on' : 'off'}`} disabled={switchDisabled} onClick={() => void toggleArmed()}>
                <span className="track-label">{state.armed ? t('common:armedOn') : t('common:armedOff')}</span>
                <span className="thumb" />
              </button>
            </div>

            <div className="state-mini-grid">
              <div><label>{t('common:connection')}</label><strong>{loading ? t('common:unavailable') : state.connection_state}</strong></div>
              <div><label>{t('common:mode')}</label><strong>{loading ? t('common:unavailable') : state.mode}</strong></div>
              <div><label>{t('common:cycle')}</label><strong>{loading ? '--' : `${state.cycle_ms.toFixed(1)} ms`}</strong></div>
              <div><label>{t('common:loss')}</label><strong>{loading ? '--' : `${state.loss_percent.toFixed(1)}%`}</strong></div>
            </div>

            <div className="joint-telemetry-list">
              {sortedJoints.map(([joint, value]) => (
                <div className="joint-row" key={joint}>
                  <span>{joint}</span>
                  <strong>{value.toFixed(3)}</strong>
                </div>
              ))}
            </div>
          </aside>

          <section className="panel stage-panel">
            <div className="stage-badge">{t('control:stage.badge')}</div>
            <h2>{t('control:stage.title')}</h2>
            <p>{t('control:stage.subtitle')}</p>
            <div className="stage-core">
              <div className="vignette" />
              <div className="halo" />
              <div className="scan-lines" />
              <div className="stage-placeholder">3D ROBOT VIEWPORT</div>
            </div>
          </section>

          <aside className="panel action-panel">
            <div className="feature-card primary primary-control">
              <h3>{t('control:cards.cartesian.title')}</h3>
              <p>{t('control:cards.cartesian.desc')}</p>
            </div>
            <div className="aux-grid mid-tier-grid">
              <div className="feature-card mid-tier"><h3>{t('control:cards.teach.title')}</h3><p>{t('control:cards.teach.desc')}</p></div>
              <div className="feature-card mid-tier"><h3>{t('control:cards.motion.title')}</h3><p>{t('control:cards.motion.desc')}</p></div>
            </div>
            <div className="aux-grid assist-tier-grid">
              <div className="feature-card assist"><h3>{t('control:cards.io.title')}</h3><p>{t('control:cards.io.desc')}</p></div>
              <div className="feature-card teach-types assist">
                <h3>{t('control:cards.teachTypes.title')}</h3>
                <div className="tag-wrap">{teachTypes.map((item) => <span key={item} className="tag">{item}</span>)}</div>
              </div>
            </div>
            <div className="feature-card sequence-card">
              <h3>{t('control:cards.sequence.title')}</h3>
              {steps.length === 0 ? <p>{t('control:cards.sequence.empty')}</p> : (
                <ul>
                  {steps.map((step) => (
                    <li key={step.id}><span>{step.type}</span><strong>{step.name}</strong></li>
                  ))}
                </ul>
              )}
            </div>
          </aside>
        </main>
      </div>
    </div>
  );
}

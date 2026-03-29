export type RequestLifecycleStatus = 'idle' | 'in_progress' | 'success' | 'failed';
export type RequestAction = 'none' | 'load' | 'enable' | 'disable' | 'recover' | 'stop' | 'clear_fault' | 'fault' | string;

export type ControlRequestStatus = {
  action: RequestAction;
  status: RequestLifecycleStatus;
  message: string | null;
};

export type RobotControlContract = {
  robot_state: 'DISCOVERING' | 'SYNCING_POSITION' | 'READY_UNARMED' | 'ARMING' | 'ARMED_HOLDING_CURRENT' | 'EXECUTING' | 'FAULT' | string;
  connection_state: string;
  armed: boolean;
  mode: string;
  cycle_ms: number;
  loss_percent: number;
  error_message: string | null;
  can_retry: boolean;
  request: ControlRequestStatus;
  joints: Record<string, number>;
};

export type ControlRequestResponse = {
  request: ControlRequestStatus;
  state: RobotControlContract;
};

export type JointResponse = { joints: Record<string, number> };
export type ModeResponse = { mode: string };

export type TeachStep = {
  id: string;
  type: 'MoveJ' | 'MoveL' | 'MoveC' | 'Wait' | 'SetIO' | string;
  name: string;
};

export type TeachSequenceResponse = { steps: TeachStep[] };

const BASE_URL = (import.meta.env.VITE_WEB_BRIDGE_BASE_URL as string | undefined) ?? 'http://localhost:8090';

async function requestJson<T>(path: string, init?: RequestInit): Promise<T> {
  const res = await fetch(`${BASE_URL}${path}`, {
    headers: { 'Content-Type': 'application/json' },
    ...init,
  });
  if (!res.ok) {
    const maybe = await res.json().catch(() => ({}));
    throw new Error(maybe?.state?.error_message ?? `${path} -> ${res.status}`);
  }
  return (await res.json()) as T;
}

export const webBridgeApi = {
  getState: () => requestJson<RobotControlContract>('/state'),
  getJoints: () => requestJson<JointResponse>('/joints'),
  getMode: () => requestJson<ModeResponse>('/mode'),
  enable: () => requestJson<ControlRequestResponse>('/enable', { method: 'POST' }),
  disable: () => requestJson<ControlRequestResponse>('/disable', { method: 'POST' }),
  recover: () => requestJson<ControlRequestResponse>('/recover', { method: 'POST' }),
  stop: () => requestJson<ControlRequestResponse>('/stop', { method: 'POST' }),
  clearFault: () => requestJson<ControlRequestResponse>('/clear_fault', { method: 'POST' }),
  fault: () => requestJson<ControlRequestResponse>('/fault', { method: 'POST' }),
  getTeachSequence: () => requestJson<TeachSequenceResponse>('/teach/sequence'),
  setTeachSequence: (steps: TeachStep[]) =>
    requestJson<TeachSequenceResponse>('/teach/sequence', {
      method: 'POST',
      body: JSON.stringify({ steps }),
    }),
};

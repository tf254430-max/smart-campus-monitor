-- Smart Campus Monitor - telemetry database (Supabase / PostgreSQL)
-- Tinka Fahad 254430, BDA 2201 CAT 2
--
-- The ESP32 POSTs {"payload": {...}} to /rest/v1/telemetry, the same JSON it
-- publishes over MQTT. The columns below are generated from that JSON, so the
-- firmware only has to send one thing and the table still has proper columns
-- to query, chart and export for training a model later.

create table if not exists public.telemetry (
  id              bigint generated always as identity primary key,
  received_at     timestamptz not null default now(),
  payload         jsonb not null,

  device_id       text    generated always as (payload->>'node') stored,
  facility        text    generated always as (payload->>'facility') stored,
  facility_key    text    generated always as (payload->>'facility_key') stored,
  status          text    generated always as (payload->>'status') stored,
  reason          text    generated always as (payload->>'reason') stored,
  low_confidence  boolean generated always as ((payload->>'low_confidence')::boolean) stored,
  uptime_s        integer generated always as ((payload->>'uptime_s')::integer) stored,

  -- null means that sensor was offline when the reading was taken
  distance_cm     real    generated always as ((payload->'telemetry'->>'distance_cm')::real) stored,
  temp_c          real    generated always as ((payload->'telemetry'->>'temp_c')::real) stored,
  humidity_pct    real    generated always as ((payload->'telemetry'->>'humidity_pct')::real) stored,
  light_lux       real    generated always as ((payload->'telemetry'->>'light_lux')::real) stored,
  motion          boolean generated always as ((payload->'telemetry'->>'motion')::boolean) stored,
  pir_ratio_30s   real    generated always as ((payload->'telemetry'->>'pir_ratio_30s')::real) stored,

  occupancy_score real    generated always as ((payload->'edge_ai'->>'occupancy_score')::real) stored,
  comfort_index   real    generated always as ((payload->'edge_ai'->>'comfort_index')::real) stored,
  spike_detected  boolean generated always as ((payload->'edge_ai'->>'spike_detected')::boolean) stored,
  sensor_faults   text    generated always as (payload->>'sensor_faults') stored,

  led_colour      text    generated always as (payload->'actuators'->>'rgb') stored,
  servo_deg       integer generated always as ((payload->'actuators'->>'servo_deg')::integer) stored,
  rssi_dbm        integer generated always as ((payload->'network'->>'rssi_dbm')::integer) stored,
  last_http_ms    integer generated always as ((payload->'network'->>'last_http_ms')::integer) stored,

  -- reject anything that is not a reading from the node
  constraint payload_has_status check (payload ? 'status' and payload ? 'facility_key')
);

create index if not exists telemetry_received_idx on public.telemetry (received_at desc);
create index if not exists telemetry_facility_idx on public.telemetry (facility_key, received_at desc);

-- the key in the firmware and the dashboard is the public "anon" key.
-- row level security lets it add and read rows, but never edit or delete them.
grant usage on schema public to anon;
grant select, insert on public.telemetry to anon;
alter table public.telemetry enable row level security;
drop policy if exists "node can insert readings" on public.telemetry;
create policy "node can insert readings" on public.telemetry for insert to anon with check (true);
drop policy if exists "anyone can read readings" on public.telemetry;
create policy "anyone can read readings" on public.telemetry for select to anon using (true);

-- newest status of each facility (what the dashboard cards show)
create or replace view public.latest_status with (security_invoker = true) as
select distinct on (facility_key) *
from public.telemetry
order by facility_key, received_at desc;

-- how often each facility was in each state (rows per status)
create or replace view public.status_summary with (security_invoker = true) as
select facility, status, count(*) as readings, max(received_at) as last_seen
from public.telemetry
group by facility, status
order by facility, status;

grant select on public.latest_status, public.status_summary to anon;

-- push new rows to the dashboard as they arrive (supabase realtime)
do $$ begin
  alter publication supabase_realtime add table public.telemetry;
exception when duplicate_object then null; end $$;

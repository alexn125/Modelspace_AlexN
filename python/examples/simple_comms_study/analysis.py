###############################################################################
# Copyright (c) ATTX LLC 2025. All Rights Reserved.
#
# This software and associated documentation (the "Software") are the 
# proprietary and confidential information of ATTX, LLC. The Software is 
# furnished under a license agreement between ATTX and the user organization 
# and may be used or copied only in accordance with the terms of the agreement.
# Refer to 'license/attx_license.adoc' for standard license terms.
#
# EXPORT CONTROL NOTICE: THIS SOFTWARE MAY INCLUDE CONTENT CONTROLLED UNDER THE
# INTERNATIONAL TRAFFIC IN ARMS REGULATIONS (ITAR) OR THE EXPORT ADMINISTRATION 
# REGULATIONS (EAR99). No part of the Software may be used, reproduced, or 
# transmitted in any form or by any means, for any purpose, without the express 
# written permission of ATTX, LLC.
###############################################################################
"""
Simple Comms Study Analysis
-----------------------
This script demonstrates how to analyze the results of a communications study.
It loads simulation data from a JSON file and summarizes the key parameters
for ground stations and communications models.

Author: Daniel Krobath
"""

import json
import pandas as pd
pd.options.display.float_format = '{:.2f}'.format
from modelspaceutils.AutoDocPy import AutoDocPy
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from astropy.time import Time
from astropy import units as u
import datetime

SECONDS_PER_DAY = 86400

# --- Load configuration data ---
with open('results/sim_data.json', 'r') as f:
    sim_data = json.load(f)

ground_stations = {name: node for name, node in sim_data.items() if node.get('class') == 'GroundStationModel'}
communications_models = {name: node for name, node in sim_data.items() if node.get('class') == 'CommunicationsDataModel'}

# --- Load simulation results ---
df = pd.read_csv('results/states.csv')

masked_prefix = "masked_"
sim_time_col = "time"
tdb_time_col = "tdb_time"
masked_cols = [col for col in df.columns if col.startswith(masked_prefix)]

def extract_access_windows(df, masked_col, sim_time_col='time', tdb_time_col='tdb_time', link_margin_col=None):
    windows = []
    access_num = 0
    in_access = False
    start_idx = None
    access_indices = []

    for idx, row in df.iterrows():
        masked = row[masked_col]
        sim_time = row[sim_time_col]
        tdb_time = row[tdb_time_col]
        if not masked and not in_access:
            in_access = True
            start_idx = idx
            access_num += 1
            access_indices = [idx]
        elif not masked and in_access:
            access_indices.append(idx)
        elif masked and in_access:
            end_idx = idx
            end_sim_time = df.loc[end_idx, sim_time_col]
            end_tdb_time = df.loc[end_idx, tdb_time_col]
            start_sim_time = df.loc[start_idx, sim_time_col]
            start_tdb_time = df.loc[start_idx, tdb_time_col]
            duration = end_sim_time - start_sim_time
            duration_minutes = duration / 60.0
            avg_link_margin = None
            if link_margin_col:
                avg_link_margin = df.loc[access_indices, link_margin_col].mean()
            windows.append({
                'Access ID': access_num,
                'Start Time (s)': start_sim_time,
                'End Time (s)': end_sim_time,
                'Start Time': tdb_seconds_to_utc(start_tdb_time),
                'End Time': tdb_seconds_to_utc(end_tdb_time),
                'Duration (s)': duration,
                'Duration (min)': duration_minutes,
                'Average Link Margin (dB)': avg_link_margin
            })
            in_access = False
            start_idx = None
            access_indices = []

    if in_access and start_idx is not None:
        end_idx = df.index[-1]
        end_sim_time = df.loc[end_idx, sim_time_col]
        end_tdb_time = df.loc[end_idx, tdb_time_col]
        start_sim_time = df.loc[start_idx, sim_time_col]
        start_tdb_time = df.loc[start_idx, tdb_time_col]
        duration = end_sim_time - start_sim_time
        duration_minutes = duration / 60.0
        avg_link_margin = None
        if link_margin_col:
            avg_link_margin = df.loc[access_indices, link_margin_col].mean()
        windows.append({
            'Access ID': access_num,
            'Start Time (s)': start_sim_time,
            'End Time (s)': end_sim_time,
            'Start Time': tdb_seconds_to_utc(start_tdb_time),
            'End Time': tdb_seconds_to_utc(end_tdb_time),
            'Duration (s)': duration,
            'Duration (min)': duration_minutes,
            'Average Link Margin (dB)': avg_link_margin
        })

    return windows

def tdb_seconds_to_utc(tdb_seconds):
    j2000_tt = Time('2000-01-01 12:00:00', scale='tt')
    time_tt = j2000_tt + tdb_seconds * u.s
    time_utc = time_tt.utc
    return time_utc.isot

# --- Create comms report ---
doc = AutoDocPy()
doc.file("comms_report.adoc")
doc.title("Communications Report")

summary_rows = [] # Store summary data for all ground stations
access_dfs = {}  # Store access_df for each ground station
windows_dict = {}  # Store windows for each ground station

for gs_name in ground_stations:
    masked_col = f"{masked_prefix}{gs_name.lower()}"
    # Find corresponding comms model for this ground station
    comm_name = None
    for cm_name in communications_models:
        if gs_name.lower() in cm_name.lower():
            comm_name = cm_name
            break
    # Get data rate in Mbps (convert from bits/sec if needed)
    if comm_name:
        bit_rate = float(communications_models[comm_name]['params']['bit_rate']['value'])
        data_rate_mbps = bit_rate / 1e6
        link_margin_col = f"link_margin_{gs_name.lower()}"
        if link_margin_col not in df.columns:
            link_margin_col = f"link_margin_{comm_name.split('_')[0].lower()}"
        if link_margin_col not in df.columns:
            link_margin_col = None

        range_col = f"range_{gs_name.lower()}"
        if range_col not in df.columns:
            range_col = f"range_{comm_name.split('_')[0].lower()}"
        if range_col not in df.columns:
            range_col = None

        range_rate_col = f"range_rate_{gs_name.lower()}"
        if range_rate_col not in df.columns:
            range_rate_col = f"range_rate_{comm_name.split('_')[0].lower()}"
        if range_rate_col not in df.columns:
            range_rate_col = None

    else:
        data_rate_mbps = None
        link_margin_col = None
        range_col = None
        range_rate_col = None

    windows = extract_access_windows(df, masked_col, sim_time_col, tdb_time_col, link_margin_col)
    for win in windows:
        win['Data Rate (Mbps)'] = data_rate_mbps
        win['Data Throughput (kB)'] = win['Duration (s)'] * data_rate_mbps * 125 if data_rate_mbps is not None else None
        win['Day'] = int(win['Start Time (s)'] // SECONDS_PER_DAY) + 1

    access_df = pd.DataFrame(windows)
    access_dfs[gs_name] = access_df
    windows_dict[gs_name] = windows

    total_passes = len(access_df)
    num_days = access_df['Day'].max()
    avg_passes_per_day = total_passes / num_days if num_days > 0 else 0
    avg_pass_duration = access_df['Duration (min)'].mean()
    avg_throughput_per_pass = access_df['Data Throughput (kB)'].mean()
    avg_minutes_per_day = access_df['Duration (min)'].sum() / num_days if num_days > 0 else 0
    avg_data_per_day = access_df['Data Throughput (kB)'].sum() / num_days if num_days > 0 else 0

    summary_rows.append({
        'Ground Station': gs_name,
        'Total Passes': total_passes,
        'Average Passes per Day': avg_passes_per_day,
        'Average Pass Duration (min)': avg_pass_duration,
        'Average Data Throughput per Pass (kB)': avg_throughput_per_pass,
        'Average Access Time per Day (min)': avg_minutes_per_day,
        'Average Data Throughput per Day (kB)': avg_data_per_day
    })

# --- Add the summary table at the top ---
summary_df = pd.DataFrame(summary_rows)

# Calculate totals across all ground stations
total_passes = summary_df['Total Passes'].sum()
avg_passes_per_day = summary_df['Average Passes per Day'].sum()
avg_pass_duration = (
    (summary_df['Average Pass Duration (min)'] * summary_df['Total Passes']).sum() / total_passes
    if total_passes > 0 else 0
)
avg_throughput_per_pass = (
    (summary_df['Average Data Throughput per Pass (kB)'] * summary_df['Total Passes']).sum() / total_passes
    if total_passes > 0 else 0
)
avg_minutes_per_day = summary_df['Average Access Time per Day (min)'].sum()
avg_data_per_day = summary_df['Average Data Throughput per Day (kB)'].sum()

total_row = {
    'Ground Station': 'Total',
    'Total Passes': total_passes,
    'Average Passes per Day': avg_passes_per_day,
    'Average Pass Duration (min)': avg_pass_duration,
    'Average Data Throughput per Pass (kB)': avg_throughput_per_pass,
    'Average Access Time per Day (min)': avg_minutes_per_day,
    'Average Data Throughput per Day (kB)': avg_data_per_day
}

# Insert the total row at the top
summary_df = pd.concat([pd.DataFrame([total_row]), summary_df], ignore_index=True)

doc.addPrimaryHeader("Ground Station Summary Table")
doc.addDataFrame(summary_df.round(2))

# --- Second loop: add per-ground-station plots and tables ---
for gs_name in ground_stations:
    access_df = access_dfs[gs_name]
    windows = windows_dict[gs_name]

    # --- Overlay plots for Range, Range Rate, and Link Margin per access ---
    comm_name = None
    for cm_name in communications_models:
        if gs_name.lower() in cm_name.lower():
            comm_name = cm_name
            break

    if comm_name:
        link_margin_col = f"link_margin_{gs_name.lower()}"
        if link_margin_col not in df.columns:
            link_margin_col = f"link_margin_{comm_name.split('_')[0].lower()}"
        if link_margin_col not in df.columns:
            link_margin_col = None

        range_col = f"range_{gs_name.lower()}"
        if range_col not in df.columns:
            range_col = f"range_{comm_name.split('_')[0].lower()}"
        if range_col not in df.columns:
            range_col = None

        range_rate_col = f"range_rate_{gs_name.lower()}"
        if range_rate_col not in df.columns:
            range_rate_col = f"range_rate_{comm_name.split('_')[0].lower()}"
        if range_rate_col not in df.columns:
            range_rate_col = None
    else:
        link_margin_col = None
        range_col = None
        range_rate_col = None

    overlay_params = [
        (range_col, 'Range (km)'),
        (range_rate_col, 'Range Rate (km/s)'),
        (link_margin_col, 'Link Margin (dB)')
    ]

    for param_col, ylabel in overlay_params:
        if param_col is None or param_col not in df.columns:
            continue
        doc.addSecondaryHeader(f"{gs_name} {ylabel} Overlay (Per Access)")
        plt.figure()
        for win in windows:
            try:
                start_idx = df.index[df[sim_time_col] == win['Start Time (s)']][0]
                end_idx = df.index[df[sim_time_col] == win['End Time (s)']][0]
            except IndexError:
                continue
            access_slice = df.iloc[start_idx:end_idx+1].copy()
            if access_slice[param_col].iloc[-1] == 0:
                access_slice = access_slice.iloc[:-1]
            shifted_time = access_slice[sim_time_col] - win['Start Time (s)']
            if 'range' in param_col:
                ydata = access_slice[param_col] / 1000.0
            elif 'range_rate' in param_col:
                ydata = access_slice[param_col] / 1000.0
            else:
                ydata = access_slice[param_col]
            plt.plot(shifted_time, ydata, label=f"Access {win['Access ID']}")
        plt.xlabel('Access Time (s)')
        plt.ylabel(ylabel)
        plt.title(f"{ylabel} per Access for {gs_name}")
        plt.grid(True)
        if len(windows) <= 10:
            plt.legend()
        doc.addMatPlotLib(plt, f"{gs_name}_{param_col}_overlay_zeroed")
        plt.close()

    # ...repeat for the "Full Simulation" overlays and daily throughput plots as before...

    doc.addSecondaryHeader(f"{gs_name} Data Transfer")
    daily_throughput = access_df.groupby('Day')['Data Throughput (kB)'].sum().reset_index()
    plt.figure()
    plt.plot(daily_throughput['Day'], daily_throughput['Data Throughput (kB)'], marker='o')
    plt.xlabel('Day')
    plt.ylabel('Total Data Throughput (kB)')
    plt.title(f'Total Data Throughput per Day for {gs_name}')
    plt.grid(True)
    plt.gca().xaxis.set_major_locator(mticker.MaxNLocator(integer=True))
    doc.addMatPlotLib(plt, f"{gs_name}_daily_throughput_chart")
    plt.close()


    doc.addPrimaryHeader(f"{gs_name} Access Tables")
    for day, day_df in access_df.groupby('Day'):
        doc.addSecondaryHeader(f"Day {day}")
        # Insert a line break after the date in the time strings
        for col in ['Start Time', 'End Time']:
            if col in day_df.columns:
                day_df[col] = day_df[col].str.replace('T', '\n')
        day_df = day_df.drop(columns=['Day', 'Start Time (s)', 'End Time (s)', 'Duration (s)'])
        doc.addDataFrame(day_df.round(2))

doc.closeDisplayDoc()
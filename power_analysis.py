#!/usr/bin/env python3
"""
Power Draw Analysis Script
Analyzes voltage and current data from CSV to calculate total power draw per mechanism.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# Read the CSV file
csv_path = "/Users/zoezhao/Downloads/logs/dcmp/dcmp-p1-power.csv"
df = pd.read_csv(csv_path)

# Replace 'null' strings with NaN
df = df.replace('null', np.nan)

# Convert all columns except Timestamp to numeric
for col in df.columns:
    if col != 'Timestamp':
        df[col] = pd.to_numeric(df[col], errors='coerce')

# Calculate power for each motor (Power = |Voltage| * |Current|)
# Shooter motors
df['shooter_influencer_power'] = (
    df['NT:/SmartDashboard/shooter/influencer/actualVoltage'].abs() *
    df['NT:/SmartDashboard/shooter/influencer/actualStatorCurrent'].abs()
)
df['shooter_follower_power'] = (
    df['NT:/SmartDashboard/shooter/follower/actualVoltage'].abs() *
    df['NT:/SmartDashboard/shooter/follower/actualStatorCurrent'].abs()
)
df['shooter_third_power'] = (
    df['NT:/SmartDashboard/shooter/third/actualVoltage'].abs() *
    df['NT:/SmartDashboard/shooter/third/actualStatorCurrent'].abs()
)

# Column motors
df['column_influencer_power'] = (
    df['NT:/SmartDashboard/column/influencer/actualVoltage'].abs() *
    df['NT:/SmartDashboard/column/influencer/actualStatorCurrent'].abs()
)
df['column_follower_power'] = (
    df['NT:/SmartDashboard/column/follower/actualVoltage'].abs() *
    df['NT:/SmartDashboard/column/follower/actualStatorCurrent'].abs()
)

# Indexer
df['indexer_power'] = (
    df['NT:/SmartDashboard/indexer/actualVoltage'].abs() *
    df['NT:/SmartDashboard/indexer/actualStatorCurrent'].abs()
)

# Roller
df['roller_power'] = (
    df['NT:/SmartDashboard/roller/actualVoltage'].abs() *
    df['NT:/SmartDashboard/roller/actualStatorCurrent'].abs()
)

# Pivot
df['pivot_power'] = (
    df['NT:/SmartDashboard/pivot/actualVoltage'].abs() *
    df['NT:/SmartDashboard/pivot/actualStatorCurrent'].abs()
)

# Drivetrain - drive and steer motors per module
for i in range(4):
    df[f'drivetrain_module{i}_drive_power'] = (
        df[f'NT:/drivetrain/Module{i}/DriveMotorVoltage'].abs() *
        df[f'NT:/drivetrain/Module{i}/DriveMotorStatorCurrent'].abs()
    )
    df[f'drivetrain_module{i}_steer_power'] = (
        df[f'NT:/drivetrain/Module{i}/SteerMotorVoltage'].abs() *
        df[f'NT:/drivetrain/Module{i}/SteerMotorStatorCurrent'].abs()
    )

# Calculate total power for each mechanism
df['shooter_total'] = (
    df['shooter_influencer_power'] + 
    df['shooter_follower_power'] + 
    df['shooter_third_power']
)
df['column_total'] = (
    df['column_influencer_power'] + 
    df['column_follower_power']
)
df['indexer_total'] = df['indexer_power']
df['roller_total'] = df['roller_power']
df['pivot_total'] = df['pivot_power']
df['drivetrain_drive_total'] = sum(df[f'drivetrain_module{i}_drive_power'] for i in range(4))
df['drivetrain_steer_total'] = sum(df[f'drivetrain_module{i}_steer_power'] for i in range(4))
df['grand_total'] = (
    df['shooter_total'] + df['column_total'] + df['indexer_total'] +
    df['roller_total'] + df['pivot_total'] +
    df['drivetrain_drive_total'] + df['drivetrain_steer_total']
)

# Create time axis (relative to start)
df['Time'] = df['Timestamp'] - df['Timestamp'].min()

# Print summary statistics
print("=" * 60)
print("POWER DRAW ANALYSIS SUMMARY")
print("=" * 60)
print("\nAverage Power Draw (Watts):")
print(f"  Shooter:           {df['shooter_total'].mean():.2f} W")
print(f"  Column:            {df['column_total'].mean():.2f} W")
print(f"  Indexer:           {df['indexer_total'].mean():.2f} W")
print(f"  Roller:            {df['roller_total'].mean():.2f} W")
print(f"  Pivot:             {df['pivot_total'].mean():.2f} W")
print(f"  Drivetrain Drive:  {df['drivetrain_drive_total'].mean():.2f} W")
print(f"  Drivetrain Steer:  {df['drivetrain_steer_total'].mean():.2f} W")
print(f"  TOTAL:             {df['grand_total'].mean():.2f} W")

print("\nMaximum Power Draw (Watts):")
print(f"  Shooter:           {df['shooter_total'].max():.2f} W")
print(f"  Column:            {df['column_total'].max():.2f} W")
print(f"  Indexer:           {df['indexer_total'].max():.2f} W")
print(f"  Roller:            {df['roller_total'].max():.2f} W")
print(f"  Pivot:             {df['pivot_total'].max():.2f} W")
print(f"  Drivetrain Drive:  {df['drivetrain_drive_total'].max():.2f} W")
print(f"  Drivetrain Steer:  {df['drivetrain_steer_total'].max():.2f} W")
print(f"  TOTAL:             {df['grand_total'].max():.2f} W")

print("\nTotal Energy Consumed (Joules):")
time_diffs = df['Time'].diff().fillna(0)
print(f"  Shooter:           {(df['shooter_total'] * time_diffs).sum():.2f} J")
print(f"  Column:            {(df['column_total'] * time_diffs).sum():.2f} J")
print(f"  Indexer:           {(df['indexer_total'] * time_diffs).sum():.2f} J")
print(f"  Roller:            {(df['roller_total'] * time_diffs).sum():.2f} J")
print(f"  Pivot:             {(df['pivot_total'] * time_diffs).sum():.2f} J")
print(f"  Drivetrain Drive:  {(df['drivetrain_drive_total'] * time_diffs).sum():.2f} J")
print(f"  Drivetrain Steer:  {(df['drivetrain_steer_total'] * time_diffs).sum():.2f} J")
print(f"  TOTAL:             {(df['grand_total'] * time_diffs).sum():.2f} J")
print("=" * 60)

# Create visualization
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), )

# Plot 1: Power over time
ax1.plot(df['Time'], df['shooter_total'], label='Shooter', linewidth=0.5, color="#FF6B6B")
ax1.plot(df['Time'], df['column_total'], label='Column', linewidth=0.5, color="#4ECDC4")
ax1.plot(df['Time'], df['indexer_total'], label='Indexer', linewidth=0.5, color="#45B7D1")
ax1.plot(df['Time'], df['roller_total'], label='Roller', linewidth=0.5, color="#FFA07A")
ax1.plot(df['Time'], df['pivot_total'], label='Pivot', linewidth=0.5, color="#98D8C8")
ax1.plot(df['Time'], df['drivetrain_drive_total'], label='Drivetrain Drive', linewidth=0.5, color="#7B68EE")
ax1.plot(df['Time'], df['drivetrain_steer_total'], label='Drivetrain Steer', linewidth=0.5, color="#B0C4DE")

ax1.set_xlabel('Time (seconds)', fontsize=12)
ax1.set_ylabel('Power Draw (Watts)', fontsize=12)
ax1.set_title('Power Draw Over Time by Mechanism', fontsize=14, fontweight='bold')
ax1.legend(loc='best')
ax1.grid(True, alpha=0.3)

# Plot 2: Average power bar chart
mechanisms = ['Shooter', 'Column', 'Indexer', 'Roller', 'Pivot', 'DT Drive', 'DT Steer']
avg_power = [
    df['shooter_total'].mean(),
    df['column_total'].mean(),
    df['indexer_total'].mean(),
    df['roller_total'].mean(),
    df['pivot_total'].mean(),
    df['drivetrain_drive_total'].mean(),
    df['drivetrain_steer_total'].mean(),
]

bars = ax2.bar(mechanisms, avg_power, color=['#FF6B6B', '#4ECDC4', '#45B7D1', '#FFA07A', '#98D8C8', '#7B68EE', '#B0C4DE'])
ax2.set_ylabel('Average Power Draw (Watts)', fontsize=12)
ax2.set_title('Average Power Draw by Mechanism', fontsize=14, fontweight='bold')
ax2.grid(True, alpha=0.3, axis='y')

# Add value labels on bars
for bar in bars:
    height = bar.get_height()
    ax2.text(bar.get_x() + bar.get_width()/2., height,
            f'{height:.2f}W',
            ha='center', va='bottom', fontsize=10, fontweight='bold')

plt.tight_layout()
plt.savefig('result_power_analysis.png', dpi=300, bbox_inches='tight')
print("\nGraph saved as 'result_power_analysis.png'")
plt.show()


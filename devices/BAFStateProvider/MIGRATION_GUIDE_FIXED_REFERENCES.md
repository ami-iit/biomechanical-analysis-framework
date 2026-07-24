# Guida: Aggiungere Fixed References al tuo RobotStateProvider_Gene01-BAFIK-PicoiFeel.xml

## Panoramica

Ora puoi aggiungere **reference fissi** (`fixed_*`) a qualsiasi task IK senza mapparlo su un sensore in `TASK_TO_SENSORS`. Questo consente a un task di funzionare anche quando non c'è un sensore disponibile.

## Quando usare i Fixed References

- **Quando un sensore non è disponibile**: Es. il tuo sistema non ha un IMU sulla testa → usa `fixed_quaternion` per mantenere la testa dritta
- **Per task di stabilità**: Es. fissa il baricentro a una posizione costante durante la camminata
- **Per esperimenti**: Testa la cinematica inversa con vincoli fissi prima di integrare sensori veri
- **Per link non critici**: Stabilizza link meno importanti con target fissi, lascia liberi quelli controllati dai sensori

## Esempi per il tuo Gene

### Esempio 1: HEAD_ORIENTATION_TASK senza sensore

**Prima** (con sensore in TASK_TO_SENSORS):
```xml
<group name="HEAD_ORIENTATION_TASK">
    <param name="type">SO3Task</param>
    <param name="robot_velocity_variable_name">robot_velocity</param>
    <param name="frame_name">head</param>
    <param name="kp_angular">20.0</param>
    <param name="node_number">62</param>
    <param name="weight">(10.0 10.0 10.0)</param>
</group>

[in TASK_TO_SENSORS]
HEAD_ORIENTATION_TASK "TransformServer::pose::head"
```

**Dopo** (con fixed reference, niente sensore):
```xml
<group name="HEAD_ORIENTATION_TASK">
    <param name="type">SO3Task</param>
    <param name="robot_velocity_variable_name">robot_velocity</param>
    <param name="frame_name">head</param>
    <param name="kp_angular">20.0</param>
    <param name="node_number">62</param>
    <param name="weight">(10.0 10.0 10.0)</param>
    <!-- La testa resta diritta: quaternione identità [w x y z] = [1 0 0 0] -->
    <param name="fixed_quaternion">(1.0 0.0 0.0 0.0)</param>
</group>

[in TASK_TO_SENSORS]
<!-- HEAD_ORIENTATION_TASK rimosso perché ha fixed_quaternion -->
```

### Esempio 2: RIGHT_TOE_TASK con fixed_wrench

**Prima** (con sensore FT reale):
```xml
<group name="RIGHT_TOE_TASK">
    <param name="type">FloorContactTask</param>
    <param name="robot_velocity_variable_name">robot_velocity</param>
    <param name="frame_name">r_sole_front</param>
    <param name="kp_linear">60.0</param>
    <param name="floor_contact_task">210</param>
    <param name="weight">(5.0 5.0 5.0)</param>
    <param name="vertical_force_threshold">50.0</param>
</group>

[in TASK_TO_SENSORS]
RIGHT_TOE_TASK "iFeelSuit::ft6D::Node#2::Front"
```

**Dopo** (con fixed_wrench, senza sensore):
```xml
<group name="RIGHT_TOE_TASK">
    <param name="type">FloorContactTask</param>
    <param name="robot_velocity_variable_name">robot_velocity</param>
    <param name="frame_name">r_sole_front</param>
    <param name="kp_linear">60.0</param>
    <param name="floor_contact_task">210</param>
    <param name="weight">(5.0 5.0 5.0)</param>
    <param name="vertical_force_threshold">50.0</param>
    <!-- Simula 100 N di forza verticale costante (appoggio a terra) -->
    <param name="fixed_wrench">(0.0 0.0 100.0 0.0 0.0 0.0)</param>
</group>

[in TASK_TO_SENSORS]
<!-- RIGHT_TOE_TASK rimosso perché ha fixed_wrench -->
```

### Esempio 3: RIGHT_HAND_POSE_TASK con fixed pose

**Prima** (con sensor mocap):
```xml
<group name="RIGHT_HAND_POSE_TASK">
    <param name="type">PoseTask</param>
    <param name="robot_velocity_variable_name">robot_velocity</param>
    <param name="frame_name">r_wrist_3</param>
    <param name="kp_angular">20.0</param>
    <param name="kp_linear">20.0</param>
    <param name="node_number">40</param>
    <param name="weight">(1.0 1.0 1.0 1.0 1.0 1.0)</param>
    <param name="rotation_matrix">(1.0 0.0 0.0 0.0 0.0 -1.0 0.0 1.0 0.0)</param>
</group>

[in TASK_TO_SENSORS]
RIGHT_HAND_POSE_TASK "TransformServer::pose::right_wrist"
```

**Dopo** (con pose fissa, senza sensore):
```xml
<group name="RIGHT_HAND_POSE_TASK">
    <param name="type">PoseTask</param>
    <param name="robot_velocity_variable_name">robot_velocity</param>
    <param name="frame_name">r_wrist_3</param>
    <param name="kp_angular">20.0</param>
    <param name="kp_linear">20.0</param>
    <param name="node_number">40</param>
    <param name="weight">(1.0 1.0 1.0 1.0 1.0 1.0)</param>
    <param name="rotation_matrix">(1.0 0.0 0.0 0.0 0.0 -1.0 0.0 1.0 0.0)</param>
    <!-- Posizione fissa: x=0.3 m, y=-0.2 m, z=0.8 m -->
    <param name="fixed_position">(0.3 -0.2 0.8)</param>
    <!-- Rotazione fissa: matrice identità (riga-major) -->
    <param name="fixed_rotation_matrix">(1.0 0.0 0.0
                                          0.0 1.0 0.0
                                          0.0 0.0 1.0)</param>
</group>

[in TASK_TO_SENSORS]
<!-- RIGHT_HAND_POSE_TASK rimosso perché ha fixed_position + fixed_rotation_matrix -->
```

## Parametri disponibili per tipo di task

### SO3Task / GravityTask (orientamento)
- **`fixed_rotation_matrix`** (9 valori, row-major) OPPURE **`fixed_quaternion`** (4: [w x y z])
- Opzionale: **`fixed_angular_velocity`** (3 valori)

### PositionTask
- **`fixed_position`** (3 valori: x y z)
- Opzionale: **`fixed_linear_velocity`** (3 valori)

### PoseTask (posizione + orientazione)
- **`fixed_position`** (3 valori) E **`fixed_rotation_matrix`** (9 valori) O **`fixed_quaternion`** (4 valori)
- Opzionale: **`fixed_linear_velocity`** (3), **`fixed_angular_velocity`** (3)

### FloorContactTask
- **`fixed_wrench`** (6 valori: [force_x force_y force_z torque_x torque_y torque_z])

## Strategie di migrazione per Gene

### Strategia 1: Ibrida (consigliata per test iniziali)
- Mantieni i sensori disponibili in TASK_TO_SENSORS
- Aggiungi fixed references solo ai task senza sensore
- Esempio: T8_TASK con sensore IMU, HEAD_ORIENTATION_TASK con fixed_quaternion

### Strategia 2: Progressiva
- Inizia con pochi task con fixed reference (es. HEAD)
- Testa la qualità del movimento IK
- Aggiungi progressivamente altri task fissi

### Strategia 3: Fallback
- Definisci ENTRAMBI sensore E fixed reference
- Quando il sensore non è disponibile, il fixed reference agisce come fallback
- (Al momento il code prioritizza il fixed reference; potresti modificare il provider per logica OR)

## File di esempio

Vedi: `devices/conf/xml/BAFStateProvider_example_with_fixed_references.xml` nel workspace biomechanical-analysis-framework per un esempio completo.

## Checklist per aggiornare il tuo Gene XML

1. **Identifica quali task non hanno sensori**: Cerca i task elencati in `<param name="tasks">` ma NON in `TASK_TO_SENSORS`
2. **Per ogni task senza sensore**:
   - Decidi il tipo di riferimento fisso appropriato
   - Aggiungi il parametro `fixed_*` corrispondente nel group del task
   - Verifica che il task NON sia in `TASK_TO_SENSORS`
3. **Test**: Lancia il provider e monitora i log per errori di validazione
4. **Sintonizzazione**: Regola i valori dei target fissi in base al comportamento desiderato

## Validazione

Il provider valida automaticamente:
- Coerenza (es. `PoseTask` richiede ENTRAMBI `fixed_position` E orientazione)
- Dimensioni (9 valori per matrice, 3 per vettore, ecc.)
- Rifiuta velocità-sola senza posizione

Errori appariranno nei log del device all'avvio.
